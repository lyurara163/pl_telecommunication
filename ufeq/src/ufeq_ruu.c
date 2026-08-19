#include "ufeq_internal.h"   /* 内部头文件：配置/工作区/矩阵运算等 */

#include <math.h>            /* sqrtf 等数学函数 */
#include <string.h>          /* memcpy / memset 等内存操作 */

/**
 * 构造 n×n 单位阵
 *
 * 【用途】
 *   Ruu 无有效输入或矩阵求逆失败时的保护回退。
 */
static void ufeq_set_identity(ufeq_cfloat_t *m, /* 输出矩阵缓冲（行主序） */
                              Ushort dim)       /* 维数 */
{
    Ushort i = 0; /* 行索引 */
    Ushort j = 0; /* 列索引 */

    for (i = 0; i < dim; ++i) {
        for (j = 0; j < dim; ++j) {
            m[i * dim + j] = (i == j) ? ufeq_cf(1.0, 0.0) : ufeq_cf(0.0, 0.0);
        }
    }
}

/**
 * 准备干扰协方差 Ruu 及其逆矩阵
 *
 * 【目的】
 *   Ruu 描述接收天线间的噪声+干扰相关。MMSE 用 Ruu^{-1} 做白化/IRC。
 *
 * 【Ruu 来源优先级】
 *   1) bypass_ruu=1：直接拷贝外部输入 request->data.ruu
 *   2) 否则若有 residual_sample：Ruu = mean(e * e^H)
 *      （只累加上三角，再 Hermitian 补全下三角）
 *   3) 否则若有 request->data.ruu：拷贝使用
 *   4) 再否则退回单位阵
 *
 * 【后处理】
 *   - rx_ant_mat_flag[i]==0：该天线行/列清零，对角置 1（天线失效保护）
 *   - ruu_add_switch：对角加载 ruu_add_coeff，并按 ruu_scale 缩放非对角
 *   - bypass_inv_ruu=1：使用外部 ruu_inv；否则 Cholesky 求逆
 *   - 求逆失败：逆矩阵退回单位阵，并计统计
 */
ufeq_status_t ufeq_prepare_ruu(const ufeq_request_t *request, /* 输入请求 */
                               ufeq_workspace_t *workspace)   /* 工作区（ruu_work / ruu_inv_work） */
{
    const ufeq_config_t *cfg = NULL; /* 配置指针 */
    Ushort n = 0;                    /* 接收天线数 = Ruu 维数 */
    Ushort i = 0;                    /* 天线索引（行） */
    Ushort j = 0;                    /* 天线索引（列） */
    ufeq_status_t st = ufeq_status_ok;
    Uint s = 0;                      /* 残差样本序号 */
    Uint n_sample = 0;               /* 残差样本组数 */
    const ufeq_cfloat_t *e = NULL;   /* 当前样本的 n 维残差向量 */
    ufeq_cfloat_t v = {0};           /* 外积项 e[i]*conj(e[j]) */

    if (request == NULL || workspace == NULL || request->param.config == NULL) {
        return ufeq_fail("ruu", "invalid_arg");
    }
    cfg = request->param.config;
    n = cfg->n_rx;

    /* 优先级 1：旁路模式，直接使用外部 Ruu */
    if (cfg->bypass_ruu) {
        if (request->data.ruu == NULL) {
            return ufeq_fail("ruu", "invalid_arg");
        }
        memcpy(workspace->ruu_work, request->data.ruu, sizeof(ufeq_cfloat_t) * (Uint)n * n);
    /* 优先级 2：从残差样本估计 Ruu = (1/N) Σ e e^H */
    } else if (request->data.residual_sample != NULL && request->data.residual_sample_count >= n) {
        n_sample = request->data.residual_sample_count / n; /* 样本组数 = 总元素数 / n_rx */
        memset(workspace->ruu_work, 0, sizeof(ufeq_cfloat_t) * (Uint)n * n);
        if (n_sample == 0) {
            ufeq_set_identity(workspace->ruu_work, n); /* 无样本，退单位阵 */
        } else {
            for (s = 0; s < n_sample; ++s) {
                e = &request->data.residual_sample[s * n]; /* 第 s 组残差向量 */
                /* 只累加上三角：Ruu[i][j] += e[i]*conj(e[j])，j >= i */
                for (i = 0; i < n; ++i) {
                    for (j = i; j < n; ++j) {
                        v = ufeq_cf_mul(e[i], ufeq_cf_conj(e[j]));
                        workspace->ruu_work[i * n + j] = ufeq_cf_add(workspace->ruu_work[i * n + j], v);
                    }
                }
            }
            /* 均值化：除以样本数 */
            for (i = 0; i < n; ++i) {
                for (j = i; j < n; ++j) {
                    workspace->ruu_work[i * n + j] =
                        ufeq_cf_scale(workspace->ruu_work[i * n + j], 1.0 / (float)n_sample);
                }
            }
        }
    /* 优先级 3：使用外部预计算的 Ruu */
    } else if (request->data.ruu != NULL) {
        memcpy(workspace->ruu_work, request->data.ruu, sizeof(ufeq_cfloat_t) * (Uint)n * n);
    /* 优先级 4：默认单位阵 */
    } else {
        ufeq_set_identity(workspace->ruu_work, n);
    }

    /* Hermitian 补全：上三角 → 全矩阵，对角保护 */
    (void)ufeq_matrix_hermitian_complete(workspace->ruu_work, n);

    /* 失效天线保护：flag[i]==0 时清零第 i 行/列，对角置 1 */
    if (request->data.rx_ant_mat_flag != NULL) {
        for (i = 0; i < n; ++i) {
            if (request->data.rx_ant_mat_flag[i] == 0) {
                for (j = 0; j < n; ++j) {
                    workspace->ruu_work[i * n + j] = ufeq_cf(0.0, 0.0);
                    workspace->ruu_work[j * n + i] = ufeq_cf(0.0, 0.0);
                }
                workspace->ruu_work[i * n + i] = ufeq_cf(1.0, 0.0);
                workspace->stats.matrix_guard_count++;
            }
        }
    }

    /* Ruu 可调：对角加载 + 非对角缩放 */
    if (!cfg->bypass_ruu_adjust && cfg->ruu_add_switch != 0) {
        for (i = 0; i < n; ++i) {
            workspace->ruu_work[i * n + i].re += cfg->ruu_add_coeff; /* 对角加载 */
            for (j = i + 1; j < n; ++j) {
                workspace->ruu_work[i * n + j] =
                    ufeq_cf_scale(workspace->ruu_work[i * n + j], cfg->ruu_scale); /* 非对角缩放 */
                workspace->ruu_work[j * n + i] = ufeq_cf_conj(workspace->ruu_work[i * n + j]);
            }
        }
    }

    /* 旁路逆矩阵：直接使用外部 ruu_inv */
    if (cfg->bypass_inv_ruu) {
        if (request->data.ruu_inv == NULL) {
            return ufeq_fail("ruu", "invalid_arg");
        }
        memcpy(workspace->ruu_inv_work, request->data.ruu_inv, sizeof(ufeq_cfloat_t) * (Uint)n * n);
        return ufeq_status_ok;
    }

    /* Cholesky 求逆 Ruu → ruu_inv_work */
    st = workspace->ops->matrix_inverse_hermitian(
        workspace->ruu_work, workspace->ruu_inv_work, n, n, 0.0);
    if (st != ufeq_status_ok) {
        workspace->stats.matrix_inverse_fail_count++;
        ufeq_set_identity(workspace->ruu_inv_work, n); /* 求逆失败，退单位阵 */
        workspace->stats.matrix_guard_count++;
    }
    return ufeq_status_ok;
}

/**
 * MRC / IRC 自适应选择
 *
 * 【目的】
 *   根据天线间干扰相关强弱，决定用对角近似（MRC）还是完整逆（IRC）。
 *
 * 【AUTO 判决】
 *   corr(i,j) = |Ruu_ij| / sqrt(Ruu_ii * Ruu_jj)
 *   取最大相关系数 max_corr：
 *     max_corr > thr + hyst  -> IRC（干扰相关强，需要联合白化）
 *     max_corr < thr - hyst  -> MRC（接近白噪声，对角近似足够）
 *     中间带（迟滞区）        -> 保持上一次 selected_mode，避免抖切
 *
 * 【模式效果】
 *   MRC：用对角功率倒数近似 Ruu^{-1}（非对角为 0），可跳过完整求逆成本
 *   IRC：保留 prepare_ruu 求出的完整逆矩阵
 */
ufeq_status_t ufeq_select_equalizer_mode(const ufeq_request_t *request, /* 输入请求 */
                                         ufeq_workspace_t *workspace)   /* 工作区（含 ruu_work） */
{
    const ufeq_config_t *cfg = NULL;
    Ushort n = 0;                      /* 接收天线数 */
    Ushort i = 0;
    Ushort j = 0;
    float max_corr = 0.0;              /* 天线间最大归一化相关系数 */
    float sum_corr = 0.0;              /* 相关系数累加（预留统计） */
    Uint corr_count = 0;               /* 相关系数对数（预留统计） */
    ufeq_equalizer_mode_t mode = ufeq_equalizer_mrc; /* 当前选定模式 */
    float den = 0.0;                   /* 归一化分母 sqrt(Ruu_ii * Ruu_jj) */
    float corr = 0.0;                  /* 单对天线相关系数 */
    ufeq_cfloat_t diag[ufeq_max_rx_ant * ufeq_max_rx_ant] = {0}; /* MRC 对角逆近似 */
    float p = 0.0;                     /* 对角功率 Ruu_ii */

    if (request == NULL || workspace == NULL || request->param.config == NULL) {
        return ufeq_fail("ruu", "invalid_arg");
    }
    cfg = request->param.config;
    n = cfg->n_rx;
    mode = cfg->equalizer_mode;

    /* AUTO 模式：扫描天线对相关系数，结合门限与迟滞判决 */
    if (mode == ufeq_equalizer_auto) {
        for (i = 0; i < n; ++i) {
            for (j = i + 1; j < n; ++j) {
                den = sqrtf(workspace->ruu_work[i * n + i].re * workspace->ruu_work[j * n + j].re);
                if (den < ufeq_diag_protect) {
                    corr = 0.0; /* 分母过小，视为无相关 */
                } else {
                    /* corr = |Ruu_ij| / sqrt(Ruu_ii * Ruu_jj) */
                    corr = sqrtf(ufeq_cf_abs2(workspace->ruu_work[i * n + j])) / den;
                }
                if (corr > max_corr) {
                    max_corr = corr;
                }
                sum_corr += corr;
                corr_count++;
            }
        }
        if (max_corr > (cfg->corr_threshold + cfg->corr_hysteresis)) {
            mode = ufeq_equalizer_irc; /* 相关强 → IRC */
        } else if (max_corr < (cfg->corr_threshold - cfg->corr_hysteresis)) {
            mode = ufeq_equalizer_mrc; /* 相关弱 → MRC */
        } else {
            mode = workspace->selected_mode; /* 迟滞区：保持上次模式 */
            if (mode != ufeq_equalizer_mrc && mode != ufeq_equalizer_irc) {
                mode = ufeq_equalizer_mrc; /* 非法历史值，默认 MRC */
            }
        }
    }

    /* MRC：构造对角逆近似 Ruu_inv ≈ diag(1/Ruu_ii) */
    if (mode == ufeq_equalizer_mrc) {
        for (i = 0; i < n; ++i) {
            p = workspace->ruu_work[i * n + i].re;
            if (p < ufeq_diag_protect) {
                p = ufeq_diag_protect; /* 对角功率下限保护 */
            }
            diag[i * n + i] = ufeq_cf(workspace->ops->reciprocal_f32(p), 0.0);
        }
        memcpy(workspace->ruu_inv_work, diag, sizeof(ufeq_cfloat_t) * (Uint)n * n);
        workspace->stats.mrc_select_count++;
    } else {
        workspace->stats.irc_select_count++; /* IRC：保留完整逆矩阵 */
    }

    workspace->selected_mode = mode;
    workspace->data_cfo_phase = 0.0; /* 重置数据 CFO 相位累积 */
    (void)sum_corr;
    (void)corr_count;
    (void)max_corr;
    return ufeq_status_ok;
}
