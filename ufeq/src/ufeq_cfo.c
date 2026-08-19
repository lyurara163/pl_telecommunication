#include "ufeq_internal.h"   /* 内部头文件：配置/工作区/复数工具等声明 */

#include <math.h>            /* 提供 sqrtf 等数学函数（本文件间接依赖 ops） */
#include <string.h>          /* 提供 memcpy 等内存操作函数 */

/**
 * DMRS 频偏前校准（CFO Pre-Correction）
 *
 * 【目的】
 *   相邻 DMRS 符号之间通常仍残留公共载波频偏（CFO）。若不先消掉，后续时域线性
 *   插值会把“相位随符号索引线性旋转”误当成信道时变，插值结果会变差。
 *   本函数在插值之前，估计并消除该公共相位斜率。
 *
 * 【算法】
 *   对相邻 DMRS 符号对 (la, lb)：
 *     1) 在所有层 / 天线 / 子载波上累加相关：
 *          acc += conj(H(la)) * H(lb)
 *     2) 相邻符号平均相位差再除以符号间隔，得到每符号相位斜率：
 *          delta_theta_pair = atan2(Im(acc), Re(acc)) / (lb - la)
 *     3) 对所有相邻对取平均，得到 slot 内统一的 delta_theta。
 *   再以 dmrs_ref_symbol 为参考符号，对每个 DMRS 符号 l 做相位回旋：
 *          H_corr(l) = H(l) * exp(-j * delta_theta * (l - l_ref))
 *
 * 【定标】
 *   输入 channel_est 按 Q(N,5) 近似：定点值 / 32 转为浮点写入 channel_work。
 *
 * 【旁路】
 *   bypass_dmrs_freq_offset=1，或仅 1 个 DMRS 符号（无法估斜率）时：
 *   只做定标转换，delta_theta 置 0，不旋转。
 *
 * 【输出】
 *   workspace->channel_work     ：校准后的 DMRS 信道（供后续插值）
 *   workspace->dmrs_delta_theta ：估计出的每符号相位斜率（弧度）
 */
ufeq_status_t ufeq_dmrs_cfo_pre_correct(const ufeq_request_t *request, /* 输入请求（含信道估计） */
                                        ufeq_workspace_t *workspace)   /* 工作区（输出 channel_work） */
{
    const ufeq_config_t *cfg = NULL; /* 指向配置结构 */
    Uint expected = 0;             /* 全 slot 信道矩阵元素个数 */
    Uint i = 0;                    /* 通用循环 / 线性下标 */
    Ushort p = 0;                    /* 发送层索引 */
    Ushort r = 0;                    /* 接收天线索引 */
    Ushort k = 0;                    /* PUSCH 子载波线性索引 */
    float delta_theta = 0.0;         /* 累加的每符号相位斜率（弧度） */
    Ushort pair_count = 0;           /* 有效相邻 DMRS 对计数 */
    Ushort la = 0;                   /* 相邻对左端 DMRS 符号 */
    Ushort lb = 0;                   /* 相邻对右端 DMRS 符号 */
    ufeq_cfloat_t acc = {0};         /* 跨层/天线/子载波的相关累加器 */
    int den = 0;                     /* 符号间隔 (lb - la) */
    Ushort rb = 0;                   /* 资源块索引 */
    Ushort re = 0;                   /* RB 内 RE 索引 */
    Uint ia = 0;                   /* H(la) 在缓冲中的线性下标 */
    Uint ib = 0;                   /* H(lb) 在缓冲中的线性下标 */
    Ushort li = 0;                   /* 当前待校正的 DMRS 符号 */
    int dsym = 0;                    /* 相对参考符号的符号偏移 (l - l_ref) */
    float phase = 0.0;               /* 回旋相位（弧度） */
    float s = 0.0;                   /* sin(phase) */
    float c = 0.0;                   /* cos(phase) */
    ufeq_cfloat_t rot = {0};         /* 旋转因子 exp(j*phase) */
    Uint idx = 0;                  /* 当前信道元素线性下标 */

    if (request == NULL || workspace == NULL || request->param.config == NULL) {
        return ufeq_fail("cfo", "invalid_arg");
    }
    cfg = request->param.config;
    expected = (Uint)cfg->n_symb_slot * cfg->n_rb * cfg->n_sc_rb * cfg->n_rx * cfg->n_layer;
    if (request->data.channel_est == NULL || request->data.channel_est_count < expected) {
        return ufeq_fail("cfo", "invalid_arg");
    }
    if (workspace->channel_work == NULL || workspace->channel_work_count < expected) {
        return ufeq_fail("cfo", "buffer_small");
    }

    /* 定点信道估计 -> 浮点工作区（Q(N,5)：除以 32 还原幅度） */
    for (i = 0; i < expected; ++i) {
        workspace->channel_work[i] = ufeq_cint32_to_cf(request->data.channel_est[i], 1.0 / 32.0);
    }
    workspace->channel_work_owns = true;

    /* 无法估计公共斜率时直接透传：旁路开关或仅 1 个 DMRS 符号 */
    if (cfg->bypass_dmrs_freq_offset || cfg->n_rs <= 1) {
        workspace->dmrs_delta_theta = 0.0;
        return ufeq_status_ok;
    }

    /* ---- 阶段 1：相邻 DMRS 对估计每符号相位斜率，再平均 ---- */
    for (i = 0; i + 1 < cfg->n_rs; ++i) {
        la = cfg->dmrs_symbol[i];
        lb = cfg->dmrs_symbol[i + 1];
        acc = ufeq_cf(0.0, 0.0); /* 清零本对相关累加器 */
        den = (int)lb - (int)la;
        if (den == 0) {
            continue; /* 两符号相同，跳过无效对 */
        }
        /* 全层/全天线/全子载波相关累加：acc += conj(H(la)) * H(lb)，提高相位估计 SNR */
        for (p = 0; p < cfg->n_layer; ++p) {
            for (r = 0; r < cfg->n_rx; ++r) {
                for (k = 0; k < cfg->m_sc_pusch; ++k) {
                    rb = (Ushort)(k / cfg->n_sc_rb);
                    re = (Ushort)(k % cfg->n_sc_rb);
                    ia = ufeq_channel_index(la, rb, re, r, p, cfg);
                    ib = ufeq_channel_index(lb, rb, re, r, p, cfg);
                    acc = ufeq_cf_add(acc, ufeq_cf_mul(ufeq_cf_conj(workspace->channel_work[ia]),
                                                       workspace->channel_work[ib]));
                }
            }
        }
        /* 本对平均相位差 / 符号间隔 → 每符号相位斜率，累加到 delta_theta */
        delta_theta += workspace->ops->atan2_f32(acc.im, acc.re) / (float)den;
        pair_count++;
    }

    if (pair_count > 0) {
        delta_theta /= (float)pair_count; /* 对所有相邻对取平均 */
    }
    workspace->dmrs_delta_theta = delta_theta;

    /* ---- 阶段 2：相对参考符号做相位回旋，消除公共 CFO ---- */
    for (i = 0; i < cfg->n_rs; ++i) {
        li = cfg->dmrs_symbol[i];
        dsym = (int)li - (int)cfg->dmrs_ref_symbol; /* 相对参考符号的偏移量 */
        phase = -delta_theta * (float)dsym;         /* 补偿相位 = -斜率 × 偏移 */
        workspace->ops->sin_cos_f32(phase, &s, &c);
        rot = ufeq_cf(c, s); /* exp(j*phase) = cos + j*sin */
        for (p = 0; p < cfg->n_layer; ++p) {
            for (r = 0; r < cfg->n_rx; ++r) {
                for (k = 0; k < cfg->m_sc_pusch; ++k) {
                    rb = (Ushort)(k / cfg->n_sc_rb);
                    re = (Ushort)(k % cfg->n_sc_rb);
                    idx = ufeq_channel_index(li, rb, re, r, p, cfg);
                    workspace->channel_work[idx] = ufeq_cf_mul(workspace->channel_work[idx], rot);
                }
            }
        }
    }
    return ufeq_status_ok;
}

/**
 * 均衡后频偏校准（Post-CFO Correction）
 *
 * 【目的】
 *   MMSE 均衡之后，仍可能存在按 OFDM 符号变化的残余相位。
 *   上层（或前端）已估好每符号补偿相位，本函数按表做复乘校正。
 *
 * 【算法】
 *   对每个 OFDM 符号 symb：
 *     phase = post_cfo_phase[symb] / 8   （Q(N,3) -> 弧度近似）
 *     对该符号下全部子载波、全部层：
 *       x_eq = x_eq * exp(j * phase)
 *
 * 【旁路】
 *   bypass_freq_offset_2=1 时直接返回，不改 equalized_work。
 *
 * 【注意】
 *   本阶段改写 workspace->equalized_work；最终导出到 result 在流水线末尾完成。
 */
ufeq_status_t ufeq_cfo_post_correct(const ufeq_request_t *request, /* 输入请求（含 post_cfo_phase） */
                                    ufeq_workspace_t *workspace)   /* 工作区（读写 equalized_work） */
{
    const ufeq_config_t *cfg = NULL; /* 指向配置结构 */
    Ushort symb = 0;                 /* 当前 OFDM 符号索引 */
    Ushort sc = 0;                   /* 当前 PUSCH 子载波索引 */
    Ushort layer = 0;                /* 当前发送层索引 */
    float phase = 0.0;               /* 本符号补偿相位（弧度） */
    float s = 0.0;                   /* sin(phase) */
    float c = 0.0;                   /* cos(phase) */
    ufeq_cfloat_t rot = {0};         /* 旋转因子 exp(j*phase) */
    Uint idx = 0;                  /* equalized_work 线性下标 */

    if (request == NULL || workspace == NULL || request->param.config == NULL) {
        return ufeq_fail("cfo", "invalid_arg");
    }
    cfg = request->param.config;
    if (cfg->bypass_freq_offset_2) {
        return ufeq_status_ok; /* 旁路：不做后频偏校正 */
    }
    if (request->data.post_cfo_phase == NULL) {
        return ufeq_fail("cfo", "invalid_arg");
    }

    for (symb = 0; symb < cfg->n_symb_slot; ++symb) {
        /* Q(N,3)：定点相位 / 8 还原为浮点弧度近似 */
        phase = ((float)request->data.post_cfo_phase[symb]) / 8.0;
        workspace->ops->sin_cos_f32(phase, &s, &c);
        rot = ufeq_cf(c, s);
        for (sc = 0; sc < cfg->m_sc_pusch; ++sc) {
            for (layer = 0; layer < cfg->n_layer; ++layer) {
                idx = ufeq_sc_index(symb, sc, layer, cfg);
                workspace->equalized_work[idx] = ufeq_cf_mul(workspace->equalized_work[idx], rot);
            }
        }
    }
    return ufeq_status_ok;
}
