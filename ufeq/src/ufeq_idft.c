#include "ufeq_internal.h"   /* 内部头文件：配置/工作区/复数工具等声明 */

#include <math.h>            /* 提供 sqrtf 等数学函数 */
#include <string.h>          /* 提供 memcpy 等内存操作函数 */

/**
 * 解传输预编码 / IDFT（仅 DFT-s-OFDM）
 *
 * 【目的】
 *   DFT-s-OFDM 发送端做过 DFT 预编码；接收端均衡后需做 IDFT 还原时域符号。
 *
 * 【步骤】
 *   每层每符号调用平台 IDFT，再按 f_t / sqrt(M) 做噪声归一化。
 *   CP-OFDM 或 bypass_idft 时透传，不改 equalized_work。
 *
 * 【算法】
 *   对每个 (symb, layer)：
 *     1) 收集 M 个子载波均衡符号 → in_buf
 *     2) 调用 ops->idft 得到 out_buf
 *     3) equalized_work[k] = out_buf[k] * f_t * (1/sqrt(M))
 *     4) 若 f_t > 1，更新 sinr_work[k] = 1/(f_t - 1)
 *
 * 【输出】
 *   workspace->equalized_work ：IDFT 并归一化后的符号
 *   workspace->sinr_work      ：相应 SINR 更新（f_t>1 时）
 */
ufeq_status_t ufeq_deprecoding(const ufeq_request_t *request, /* 输入请求（含波形配置） */
                               ufeq_workspace_t *workspace)   /* 工作区（读写 equalized/sinr） */
{
    const ufeq_config_t *cfg = NULL; /* 指向配置结构 */
    Ushort symb = 0;                 /* 当前 OFDM 符号索引 */
    Ushort layer = 0;                /* 当前发送层索引 */
    Ushort k = 0;                    /* 子载波索引 */
    float inv_sqrt_m = 0.0;          /* 1/sqrt(M_sc_pusch)，IDFT 归一化因子 */
    ufeq_cfloat_t in_buf[4096] = {0};  /* IDFT 输入缓冲（频域） */
    ufeq_cfloat_t out_buf[4096] = {0}; /* IDFT 输出缓冲（时域） */
    float ft = 0.0;                  /* 当前 (symb, layer) 的噪声缩放因子 f_t */
    ufeq_status_t st = ufeq_status_ok; /* IDFT 调用返回状态 */
    Uint idx = 0;                  /* equalized_work / sinr_work 线性下标 */

    if (request == NULL || workspace == NULL || request->param.config == NULL) {
        return ufeq_fail("idft", "invalid_arg");
    }
    cfg = request->param.config;

    /* CP-OFDM 或未启用 IDFT：透传，不做解预编码 */
    if (cfg->bypass_idft || cfg->waveform == ufeq_waveform_cp_ofdm) {
        return ufeq_status_ok;
    }

    inv_sqrt_m = 1.0 / sqrtf((float)cfg->m_sc_pusch); /* 1/sqrt(M) 标准 IDFT 归一化 */
    for (symb = 0; symb < cfg->n_symb_slot; ++symb) {
        for (layer = 0; layer < cfg->n_layer; ++layer) {
            if (cfg->m_sc_pusch > 4096) {
                return ufeq_fail("idft", "unsupported_length"); /* 超出栈缓冲上限 */
            }
            /* 收集本符号本层的 M 个频域均衡符号到 in_buf */
            for (k = 0; k < cfg->m_sc_pusch; ++k) {
                in_buf[k] = workspace->equalized_work[ufeq_sc_index(symb, k, layer, cfg)];
            }
            st = workspace->ops->idft(in_buf, out_buf, cfg->m_sc_pusch);
            if (st != ufeq_status_ok) {
                return st; /* 平台 IDFT 失败，向上返回 */
            }
            ft = workspace->f_t[symb * cfg->n_layer + layer]; /* 本 (symb, layer) 的 f_t */
            for (k = 0; k < cfg->m_sc_pusch; ++k) {
                idx = ufeq_sc_index(symb, k, layer, cfg);
                /* 时域符号 = IDFT 输出 × f_t / sqrt(M) */
                workspace->equalized_work[idx] = ufeq_cf_scale(out_buf[k], ft * inv_sqrt_m);
                if (workspace->f_t[symb * cfg->n_layer + layer] > 1.0) {
                    /* f_t > 1 时更新 SINR：1/(f_t - 1) */
                    workspace->sinr_work[idx] =
                        workspace->ops->reciprocal_f32(workspace->f_t[symb * cfg->n_layer + layer] - 1.0);
                }
            }
        }
    }
    return ufeq_status_ok;
}
