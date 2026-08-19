#include "ufeq_internal.h"   /* 内部头文件：配置/工作区/矩阵运算等 */

#include <math.h>            /* fabsf 等数学函数 */
#include <string.h>          /* memset 等内存操作 */

/**
 * MMSE 均衡（单 RE 核心）
 *
 * 【目的】
 *   在每个 (OFDM符号, 子载波) 上，用插值后的信道 H 与干扰协方差逆 Ruu^{-1}，
 *   估计各层发送符号 x_hat，并给出对应 SINR（rho）。
 *
 * 【单 RE 计算步骤】
 *   设 y 为 n_rx 维接收向量，h 为 n_rx x n_layer 信道矩阵：
 *     1) h_bar^H = h^H * Ruu^{-1}          （白化后的匹配滤波）
 *     2) r_hh    = h_bar^H * h + I         （层间相关，对角可加保护加载）
 *     3) x_tilde = h_bar^H * y             （充分统计量）
 *     4) x_hat   = r_hh^{-1} * x_tilde     （MMSE 解）
 *     5) rho     = 1/diag(r_hh_inv) - 1    （等效 SINR）
 *
 * 【波形差异】
 *   CP-OFDM   ：输出乘 f_cp = (1+rho)/rho，做幅度归一
 *   DFT-s-OFDM：输出保留 x_hat，并累计 f_dft=1-diag，符号末算 f_t 供 IDFT 用
 *
 * 【特殊处理】
 *   - 求逆失败：回退单位阵，并计 matrix_inverse_fail_count
 *   - rho / diag 超界：钳到保护范围，计 sinr_guard_count
 *   - DC 子载波：x_hat 与 rho 清零
 *   - bypass_symbol_process：跳过真正均衡，透传 rx 并填最小 SINR
 *
 * 【注意】
 *   本函数会先写一份定点 equalized/sinr 到 result；流水线后级还可能改浮点工作区，
 *   最终以 ufeq_process 末尾导出为准。
 */
ufeq_status_t ufeq_mmse_equalize(const ufeq_request_t *request, /* 输入请求 */
                                 ufeq_workspace_t *workspace,   /* 工作区（信道/Ruu/中间量） */
                                 ufeq_result_t *result)         /* 输出结果（定点均衡/SINR） */
{
    const ufeq_config_t *cfg = NULL;
    Ushort symb = 0;                 /* 当前 OFDM 符号索引 */
    Ushort sc = 0;                   /* 当前 PUSCH 子载波线性索引 */
    Ushort rx = 0;                   /* 接收天线索引 */
    Ushort layer = 0;                /* 发送层索引 */
    Ushort p = 0;                    /* 层索引（矩阵运算） */
    Ushort q = 0;                    /* 层索引（矩阵运算） */
    Ushort n_rx = 0;                 /* 接收天线数 */
    Ushort n_layer = 0;              /* 发送层数 */
    float scale_out = 0.0;           /* 均衡输出定点缩放因子 2^n_mmse_shift */
    Uint need = 0;                 /* 旁路模式所需元素数 */
    float f_dft_sum[ufeq_max_layer] = {0}; /* DFT-s-OFDM：符号内各层 f_dft 累加 */
    Ushort rb = 0;                   /* 资源块索引 */
    Ushort re = 0;                   /* RB 内 RE 索引 */
    ufeq_cfloat_t h[ufeq_max_rx_ant * ufeq_max_layer] = {0}; /* 当前 RE 信道矩阵 [rx][layer] */
    ufeq_cfloat_t y[ufeq_max_rx_ant] = {0};                  /* 当前 RE 接收向量 */
    ufeq_cfloat_t h_h[ufeq_max_layer * ufeq_max_rx_ant] = {0}; /* h^H，存为 [layer][rx] */
    ufeq_status_t st = ufeq_status_ok;
    Uint dci = 0;                    /* DC 子载波列表索引 */
    Uint y_idx = 0;                /* 接收数据线性下标 */
    Uint h_idx = 0;                /* 信道估计线性下标 */
    Uint oidx = 0;                 /* 均衡输出线性下标 */
    ufeq_cfloat_t sum = {0};         /* 矩阵内积累加 */
    float diag = 0.0;                /* r_hh_inv 对角元素（与 SINR 相关） */
    float rho = 0.0;                 /* 等效 SINR = 1/diag - 1 */
    float f_dft = 0.0;               /* DFT-s-OFDM 缩放因子 1 - diag */
    float f_cp = 0.0;                /* CP-OFDM 幅度归一因子 (1+rho)/rho */
    float mean_f = 0.0;              /* 符号内 f_dft 均值 */
    float ft = 0.0;                  /* DFT-s-OFDM 符号级 IDFT 缩放 f_t */

    if (request == NULL || workspace == NULL || result == NULL || request->param.config == NULL) {
        return ufeq_fail("equalizer", "invalid_arg");
    }
    cfg = request->param.config;
    n_rx = cfg->n_rx;
    n_layer = cfg->n_layer;
    scale_out = (float)(1 << cfg->n_mmse_shift);

    /* 旁路符号处理：直接透传预均衡输入，SINR 填最小值 */
    if (cfg->bypass_symbol_process) {
        need = (Uint)cfg->n_symb_slot * cfg->m_sc_pusch * cfg->n_layer;
        if (result->equalized_capacity < need || result->sinr_capacity < need) {
            return ufeq_fail("equalizer", "buffer_small");
        }
        for (sc = 0; sc < need; ++sc) {
            workspace->equalized_work[sc] = ufeq_cint32_to_cf(request->data.pre_equalized[sc], 1.0);
            workspace->sinr_work[sc] = ufeq_rho_min;
        }
        workspace->equalized_count = need;
        return ufeq_status_ok;
    }

    /* 逐 OFDM 符号、逐 RE 做 MMSE 均衡 */
    for (symb = 0; symb < cfg->n_symb_slot; ++symb) {
        memset(f_dft_sum, 0, sizeof(f_dft_sum)); /* 新符号，重置 f_dft 累加 */

        for (sc = 0; sc < cfg->m_sc_pusch; ++sc) {
            rb = (Ushort)(sc / cfg->n_sc_rb); /* 线性子载波 → RB 索引 */
            re = (Ushort)(sc % cfg->n_sc_rb); /* 线性子载波 → RB 内 RE 索引 */

            /* 组装当前 RE 的观测 y[n_rx] 与信道 h[n_rx × n_layer] */
            for (rx = 0; rx < n_rx; ++rx) {
                y_idx = ufeq_rx_index(symb, rb, re, rx, cfg);
                y[rx] = workspace->rx_work[y_idx];
                for (layer = 0; layer < n_layer; ++layer) {
                    h_idx = ufeq_channel_index(symb, rb, re, rx, layer, cfg);
                    h[rx * n_layer + layer] = workspace->channel_interp[h_idx];
                }
            }

            /* 构造 h^H（共轭转置），存为 h_h[layer][rx] */
            for (p = 0; p < n_layer; ++p) {
                for (rx = 0; rx < n_rx; ++rx) {
                    h_h[p * n_rx + rx] = ufeq_cf_conj(h[rx * n_layer + p]);
                }
            }

            /* 步骤1：h_bar^H = h^H * Ruu^{-1}（白化匹配滤波） */
            (void)ufeq_matrix_mul(h_h, n_layer, n_rx, workspace->ruu_inv_work, n_rx, workspace->h_bar_h);

            /* 步骤2：r_hh = h_bar^H * h + I（对角加 ufeq_rhh_diag_load 保护） */
            for (p = 0; p < n_layer; ++p) {
                for (q = 0; q < n_layer; ++q) {
                    sum = ufeq_cf(0.0, 0.0);
                    for (rx = 0; rx < n_rx; ++rx) {
                        /* r_hh[p][q] += h_bar_h[p][rx] * h[rx][q] */
                        sum = ufeq_cf_add(sum,
                                          ufeq_cf_mul(workspace->h_bar_h[p * n_rx + rx],
                                                      h[rx * n_layer + q]));
                    }
                    if (p == q) {
                        sum.re += 1.0 + ufeq_rhh_diag_load; /* 对角单位阵 + 保护加载 */
                    }
                    workspace->r_hh[p * n_layer + q] = sum;
                }
            }

            /* 步骤3：求 r_hh 的逆（Cholesky） */
            st = workspace->ops->matrix_inverse_hermitian(
                workspace->r_hh, workspace->r_hh_inv, n_layer, n_layer, 0.0);
            if (st != ufeq_status_ok) {
                workspace->stats.matrix_inverse_fail_count++;
                /* 求逆失败：回退单位阵 */
                for (p = 0; p < n_layer; ++p) {
                    for (q = 0; q < n_layer; ++q) {
                        workspace->r_hh_inv[p * n_layer + q] =
                            (p == q) ? ufeq_cf(1.0, 0.0) : ufeq_cf(0.0, 0.0);
                    }
                }
            }

            /* 步骤4：匹配滤波 x_tilde = h_bar^H * y */
            for (p = 0; p < n_layer; ++p) {
                sum = ufeq_cf(0.0, 0.0);
                for (rx = 0; rx < n_rx; ++rx) {
                    sum = ufeq_cf_add(sum, ufeq_cf_mul(workspace->h_bar_h[p * n_rx + rx], y[rx]));
                }
                workspace->x_tilde[p] = sum;
            }

            /* 步骤5：MMSE 解 x_hat = r_hh_inv * x_tilde */
            (void)ufeq_matrix_mul(workspace->r_hh_inv, n_layer, n_layer, workspace->x_tilde, 1, workspace->x_hat);

            /* 步骤6：计算 SINR、波形相关缩放，写入工作区 */
            for (p = 0; p < n_layer; ++p) {
                diag = workspace->r_hh_inv[p * n_layer + p].re; /* 逆矩阵对角 → 与 SINR 相关 */
                oidx = ufeq_sc_index(symb, sc, p, cfg);
                if (diag < ufeq_diag_protect) {
                    diag = ufeq_diag_protect;
                    workspace->stats.sinr_guard_count++;
                }
                rho = workspace->ops->reciprocal_f32(diag) - 1.0; /* SINR = 1/diag - 1 */
                if (rho < ufeq_rho_min) {
                    rho = ufeq_rho_min;
                    workspace->stats.sinr_guard_count++;
                }
                if (rho > ufeq_rho_max) {
                    rho = ufeq_rho_max;
                    workspace->stats.sinr_guard_count++;
                }
                f_dft = 1.0 - diag; /* DFT-s-OFDM 逐 RE 缩放因子 */
                if (fabsf(rho) < ufeq_diag_protect) {
                    f_cp = 1.0; /* rho 过小，不做 CP 归一 */
                } else {
                    f_cp = workspace->ops->reciprocal_f32(rho) + 1.0; /* f_cp = (1+rho)/rho */
                }

                if (cfg->waveform == ufeq_waveform_cp_ofdm) {
                    workspace->equalized_work[oidx] = ufeq_cf_scale(workspace->x_hat[p], f_cp);
                } else {
                    workspace->equalized_work[oidx] = workspace->x_hat[p]; /* DFT-s-OFDM 保留原 x_hat */
                    f_dft_sum[p] += f_dft; /* 符号内累加 f_dft */
                }
                workspace->sinr_work[oidx] = rho;
                if (workspace->f_dft != NULL) {
                    workspace->f_dft[oidx] = f_dft;
                }
                if (workspace->f_cp != NULL) {
                    workspace->f_cp[oidx] = f_cp;
                }
            }

            /* DC 子载波处理：均衡输出与 SINR 清零 */
            for (dci = 0; dci < request->data.dc_count; ++dci) {
                if (request->data.dc_index[dci] == sc) {
                    for (p = 0; p < n_layer; ++p) {
                        oidx = ufeq_sc_index(symb, sc, p, cfg);
                        workspace->equalized_work[oidx] = ufeq_cf(0.0, 0.0);
                        workspace->sinr_work[oidx] = 0.0;
                    }
                } else if (request->data.dc_index[dci] >= cfg->m_sc_pusch) {
                    workspace->stats.invalid_dc_index_count++; /* DC 索引越界统计 */
                }
            }
        }

        /* DFT-s-OFDM：符号末计算 f_t = 1/mean(f_dft)，供后续 IDFT 缩放 */
        if (cfg->waveform == ufeq_waveform_dft_s_ofdm) {
            for (p = 0; p < n_layer; ++p) {
                mean_f = f_dft_sum[p] / (float)cfg->m_sc_pusch;
                if (mean_f < ufeq_diag_protect) {
                    mean_f = ufeq_diag_protect;
                }
                ft = workspace->ops->reciprocal_f32(mean_f); /* f_t = 1/mean_f */
                if (ft <= 1.0) {
                    ft = 1.0 + ufeq_diag_protect;
                    workspace->stats.sinr_guard_count++;
                }
                workspace->f_t[symb * n_layer + p] = ft;
            }
        }
    }

    workspace->equalized_count = (Uint)cfg->n_symb_slot * cfg->m_sc_pusch * cfg->n_layer;

    /* 浮点工作区 → 定点 result 导出 */
    if (result->equalized_capacity < workspace->equalized_count
        || result->sinr_capacity < workspace->equalized_count) {
        return ufeq_fail("equalizer", "buffer_small");
    }
    for (sc = 0; sc < workspace->equalized_count; ++sc) {
        result->equalized_data[sc] = ufeq_cf_to_cint32(workspace->equalized_work[sc], scale_out);
        result->sinr[sc] = (int32_t)(workspace->sinr_work[sc] * (float)(1 << cfg->n_sinr_shift) + 0.5);
    }
    result->equalized_count = workspace->equalized_count;
    result->sinr_count = workspace->equalized_count;
    result->selected_mode = workspace->selected_mode;
    (void)scale_out;
    return ufeq_status_ok;
}
