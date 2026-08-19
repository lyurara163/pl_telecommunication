#include "ufeq_internal.h"   /* 内部头文件：配置/工作区/定标工具等 */

#include <math.h>            /* sqrtf / fabsf 等数学函数 */
#include <string.h>          /* memset 等内存操作 */

/**
 * 软解调：均衡符号 → LLR 软比特
 *
 * 【目的】
 *   按码字调制阶数，将均衡后的复符号与 SINR 转换为 int8 软比特（LLR），
 *   供后续解扰与译码使用。
 *
 * 【定标流程】
 *   星座距离 d 由 Q(N,1) 定点常数经 n_mmse_shift 反量化得到；
 *   LLR 经 *2 → >> n_sb_shift → 四舍五入 → 饱和到 [-127, 127]。
 *
 * 【SINR 更新】
 *   rho 超过 (sinr_threshold_alpha/256)*th_mcs 时钳到门限，避免高 SINR 过饱和。
 */

/* Q(N,1) 定点星座距离常数（与 n_mmse_shift 配合反量化） */
#define UFEQ_D16_Q1   20724   /* 16QAM 最小距离 d */
#define UFEQ_D64_Q1   10112   /* 64QAM 最小距离 d */
#define UFEQ_D64_2_Q1 20225   /* 64QAM 2d（预留） */
#define UFEQ_D64_3_Q1 30337   /* 64QAM 3d（预留） */
#define UFEQ_D256_Q1  5029    /* 256QAM 最小距离 d */
#define UFEQ_D256_2_Q1 10059  /* 256QAM 2d（预留） */
#define UFEQ_D256_3_Q1 15088  /* 256QAM 3d（预留） */
#define UFEQ_D256_4_Q1 20118  /* 256QAM 4d（预留） */
#define UFEQ_D256_5_Q1 25147  /* 256QAM 5d（预留） */
#define UFEQ_D256_6_Q1 30176  /* 256QAM 6d（预留） */
#define UFEQ_D256_7_Q1 35206  /* 256QAM 7d（预留） */

/**
 * 符号函数：x>0→1，x<0→-1，x=0→0
 */
static float ufeq_sgn(float x)
{
    if (x > 0.0) {
        return 1.0;
    }
    if (x < 0.0) {
        return -1.0;
    }
    return 0.0;
}

/**
 * SINR 门限更新：rho 超过动态门限时钳到门限值
 *
 * 【公式】
 *   th_update = (sinr_threshold_alpha / 256) * th_mcs，钳到 [10, 32767]
 *   若 rho >= th_update，返回 th_update；否则返回 rho
 */
static float ufeq_update_rho(float rho_in, const ufeq_config_t *cfg)
{
    float th_update = (cfg->sinr_threshold_alpha / 256.0) * cfg->th_mcs; /* 动态 SINR 上限 */
    if (th_update < 10.0) {
        th_update = 10.0;
    }
    if (th_update > 32767.0) {
        th_update = 32767.0;
    }
    return (rho_in < th_update) ? rho_in : th_update;
}

/**
 * LLR 定点打包：浮点 LLR → int8 软比特
 *
 * 【定标】
 *   scaled = llr * 2 / 2^n_sb_shift → 四舍五入 → 饱和到 [-127, 127]
 */
static int8_t ufeq_pack_llr(float llr, Uchar n_sb_shift, ufeq_stats_t *stats)
{
    int32_t v = 0;
    float scaled = 0.0;

    scaled = llr;
    scaled *= 2.0; /* 设计：N 位运算后左移 1 位 */
    if (n_sb_shift > 0) {
        scaled /= (float)(1 << n_sb_shift); /* 右移 n_sb_shift */
    }
    v = (int32_t)(scaled >= 0.0 ? scaled + 0.5 : scaled - 0.5); /* 四舍五入 */
    if (v > 127 || v < -127) {
        stats->saturation_count++; /* 饱和统计 */
    }
    return ufeq_saturate_s8_sym(v);
}

/**
 * Q(N,1) 定点常数反量化为浮点距离 d
 *
 * 【公式】
 *   右移 (n_mmse_shift - 1) 后乘 2^-15 再乘 2^n_mmse_shift
 */
static float ufeq_d_from_q1(int32_t q1, Uchar n_mmse_shift)
{
    Uchar sh = 0;       /* 右移位数 */
    float scale = 0.0;  /* Q15 缩放 2^-15 */
    float v = 0.0;

    sh = (n_mmse_shift > 1) ? (Uchar)(n_mmse_shift - 1) : 0;
    scale = 1.0 / (float)(1 << 15);
    v = (float)ufeq_shift_right_round_s32(q1, sh);
    return v * scale * (float)(1 << n_mmse_shift);
}

/**
 * 16QAM 星座最小距离（供 stage-2 对齐使用）
 */
float ufeq_demod_distance_16qam(Uchar n_mmse_shift)
{
    return ufeq_d_from_q1(UFEQ_D16_Q1, n_mmse_shift);
}

/**
 * 64QAM 星座最小距离（供 stage-2 对齐使用）
 */
float ufeq_demod_distance_64qam(Uchar n_mmse_shift)
{
    return ufeq_d_from_q1(UFEQ_D64_Q1, n_mmse_shift);
}

/**
 * 256QAM 星座最小距离（供 stage-2 对齐使用）
 */
float ufeq_demod_distance_256qam(Uchar n_mmse_shift)
{
    return ufeq_d_from_q1(UFEQ_D256_Q1, n_mmse_shift);
}

/**
 * 高阶 QAM MSB 比特度量（64/256QAM bit0/bit1 共用）
 *
 * 【算法】
 *   从最高星座层 n=max_level 向下搜索，若 |x| >= n*d：
 *     LLR ∝ ((n+1)*x - T_n*sgn(x)*d) * rho，T_n = n*(n+1)/2
 *   否则退化为 x * rho
 */
static float ufeq_qam_msb_metric(float x, float d, float rho, int max_level)
{
    float ax = fabsf(x);  /* |x| */
    float s = ufeq_sgn(x); /* 符号 */
    int n = 0;            /* 当前星座层 */
    float t = 0.0;        /* T_n = n*(n+1)/2 */

    for (n = max_level; n >= 1; --n) {
        if (ax >= (float)n * d) {
            /* LLR = ((n+1)*x - T_n*sgn(x)*d) * rho */
            t = (float)(n * (n + 1) / 2);
            return ((float)(n + 1) * x - t * s * d) * rho;
        }
    }
    return x * rho; /* 最内层，退化为 BPSK 型度量 */
}

/**
 * π/2-BPSK 软解调
 *
 * 【要点】
 *   偶数符号取 Re+Im，奇数符号取 Im-Re（π/2 旋转等价）
 */
static void ufeq_demod_pi2_bpsk(ufeq_cfloat_t x, float rho, Uint idx, int8_t *out,
                                Uchar n_sb_shift, ufeq_stats_t *stats)
{
    float v = ((idx & 1) == 0) ? (x.re + x.im) : (x.im - x.re);
    out[0] = ufeq_pack_llr(v * rho, n_sb_shift, stats);
}

/**
 * QPSK 软解调：I/Q 两比特独立 LLR
 */
static void ufeq_demod_qpsk(ufeq_cfloat_t x, float rho, int8_t *out,
                            Uchar n_sb_shift, ufeq_stats_t *stats)
{
    out[0] = ufeq_pack_llr(x.re * rho, n_sb_shift, stats); /* bit0：实部 */
    out[1] = ufeq_pack_llr(x.im * rho, n_sb_shift, stats); /* bit1：虚部 */
}

/**
 * 16QAM 软解调：4 比特（MSB I/Q + LSB I/Q）
 *
 * 【LSB 公式】
 *   LLR_LSB = (d - |x|) * rho
 */
static void ufeq_demod_16qam(ufeq_cfloat_t x, float rho, float d, int8_t *out,
                             Uchar n_sb_shift, ufeq_stats_t *stats)
{
    out[0] = ufeq_pack_llr(x.re * rho, n_sb_shift, stats);
    out[1] = ufeq_pack_llr(x.im * rho, n_sb_shift, stats);
    out[2] = ufeq_pack_llr((d - fabsf(x.re)) * rho, n_sb_shift, stats);
    out[3] = ufeq_pack_llr((d - fabsf(x.im)) * rho, n_sb_shift, stats);
}

/**
 * 64QAM bit2 度量（|x| 域分段线性）
 */
static float ufeq_64qam_b2(float a, float d, float rho)
{
    if (a >= 3.0 * d) {
        return (5.0 * d - 2.0 * a) * rho;
    }
    if (a >= d) {
        return (2.0 * d - a) * rho;
    }
    return (3.0 * d - 2.0 * a) * rho;
}

/**
 * 64QAM 软解调：6 比特
 */
static void ufeq_demod_64qam(ufeq_cfloat_t x, float rho, float d, int8_t *out,
                             Uchar n_sb_shift, ufeq_stats_t *stats)
{
    float ar = fabsf(x.re);
    float ai = fabsf(x.im);
    out[0] = ufeq_pack_llr(ufeq_qam_msb_metric(x.re, d, rho, 3), n_sb_shift, stats);
    out[1] = ufeq_pack_llr(ufeq_qam_msb_metric(x.im, d, rho, 3), n_sb_shift, stats);
    out[2] = ufeq_pack_llr(ufeq_64qam_b2(ar, d, rho), n_sb_shift, stats);
    out[3] = ufeq_pack_llr(ufeq_64qam_b2(ai, d, rho), n_sb_shift, stats);
    out[4] = ufeq_pack_llr((d - fabsf(ar - 2.0 * d)) * rho, n_sb_shift, stats);
    out[5] = ufeq_pack_llr((d - fabsf(ai - 2.0 * d)) * rho, n_sb_shift, stats);
}

/**
 * 256QAM bit2 度量（|x| 域扩展分段）
 */
static float ufeq_256qam_b2(float a, float d, float rho)
{
    if (a >= 7.0 * d) {
        return (22.0 * d - 4.0 * a) * rho;
    }
    if (a >= 6.0 * d) {
        return (15.0 * d - 3.0 * a) * rho;
    }
    if (a >= 5.0 * d) {
        return (9.0 * d - 2.0 * a) * rho;
    }
    if (a >= 3.0 * d) {
        return (4.0 * d - a) * rho;
    }
    if (a >= 2.0 * d) {
        return (7.0 * d - 2.0 * a) * rho;
    }
    if (a >= d) {
        return (9.0 * d - 3.0 * a) * rho;
    }
    return (10.0 * d - 4.0 * a) * rho;
}

/**
 * 256QAM bit4 度量（|x| 域分段）
 */
static float ufeq_256qam_b4(float a, float d, float rho)
{
    if (a >= 7.0 * d) {
        return (13.0 * d - 2.0 * a) * rho;
    }
    if (a >= 5.0 * d) {
        return (6.0 * d - a) * rho;
    }
    if (a >= 4.0 * d) {
        return (11.0 * d - 2.0 * a) * rho;
    }
    if (a >= 3.0 * d) {
        return (2.0 * a - 5.0 * d) * rho;
    }
    if (a >= d) {
        return (a - 2.0 * d) * rho;
    }
    return (2.0 * a - 3.0 * d) * rho;
}

/**
 * 256QAM 软解调：8 比特
 */
static void ufeq_demod_256qam(ufeq_cfloat_t x, float rho, float d, int8_t *out,
                              Uchar n_sb_shift, ufeq_stats_t *stats)
{
    float ar = fabsf(x.re);
    float ai = fabsf(x.im);
    out[0] = ufeq_pack_llr(ufeq_qam_msb_metric(x.re, d, rho, 7), n_sb_shift, stats);
    out[1] = ufeq_pack_llr(ufeq_qam_msb_metric(x.im, d, rho, 7), n_sb_shift, stats);
    out[2] = ufeq_pack_llr(ufeq_256qam_b2(ar, d, rho), n_sb_shift, stats);
    out[3] = ufeq_pack_llr(ufeq_256qam_b2(ai, d, rho), n_sb_shift, stats);
    out[4] = ufeq_pack_llr(ufeq_256qam_b4(ar, d, rho), n_sb_shift, stats);
    out[5] = ufeq_pack_llr(ufeq_256qam_b4(ai, d, rho), n_sb_shift, stats);
    out[6] = ufeq_pack_llr((d - fabsf(fabsf(ar - 4.0 * d) - 2.0 * d)) * rho, n_sb_shift, stats);
    out[7] = ufeq_pack_llr((d - fabsf(fabsf(ai - 4.0 * d) - 2.0 * d)) * rho, n_sb_shift, stats);
}

/**
 * 按码字调制阶数输出软比特到 soft_bit_tmp
 *
 * 【目的】
 *   遍历各码字符号，按调制方式调用对应解调器，写入 workspace->soft_bit_tmp。
 *
 * 【双码字】
 *   写入 soft_bit_offset[count] / soft_bit_count[count]，供解扰分段使用。
 *
 * 【星座距离】
 *   d16 = 2/sqrt(10)，d64 = 2/sqrt(42)，d256 = 2/sqrt(170)
 */
ufeq_status_t ufeq_demodulate(const ufeq_request_t *request, ufeq_workspace_t *workspace)
{
    const ufeq_config_t *cfg = NULL;
    ufeq_modulation_t mod = ufeq_mod_qpsk; /* 当前码字调制方式 */
    Uint qm = 0;                             /* 每符号比特数 */
    Uint i = 0;                            /* 符号序号 */
    Uint n = 0;                            /* 当前码字符号数 */
    Uint out = 0;                          /* soft_bit_tmp 写入偏移 */
    float d16 = 0.0;                         /* 16QAM 最小距离 */
    float d64 = 0.0;                         /* 64QAM 最小距离 */
    float d256 = 0.0;                        /* 256QAM 最小距离 */
    Uchar cw = 0;                            /* 码字索引 */
    Uchar cw_count = 0;                      /* 有效码字数 */
    float rho = 0.0;                         /* 当前符号 SINR（经门限更新） */
    ufeq_cfloat_t x = {0};                   /* 当前均衡符号 */
    int8_t *dst = NULL;                      /* 当前符号 LLR 输出指针 */

    if (request == NULL || workspace == NULL || request->param.config == NULL) {
        return ufeq_fail("demod", "invalid_arg");
    }
    cfg = request->param.config;
    if (cfg->bypass_demod) {
        return ufeq_status_ok; /* 旁路解调，直接返回 */
    }

    /* 标准 NR 归一化星座最小距离 */
    d16 = 2.0 / sqrtf(10.0);
    d64 = 2.0 / sqrtf(42.0);
    d256 = 2.0 / sqrtf(170.0);

    /* 初始化各码字软比特偏移与计数 */
    workspace->soft_bit_offset[0] = 0;
    workspace->soft_bit_offset[1] = 0;
    workspace->soft_bit_count[0] = 0;
    workspace->soft_bit_count[1] = 0;

    /* 确定有效码字数 */
    cw_count = workspace->effective_codeword_count;
    if (cw_count == 0) {
        cw_count = cfg->codeword_count;
    }
    if (cw_count == 0) {
        cw_count = 1;
    }
    /* 逐码字解调 */
    for (cw = 0; cw < cw_count && cw < ufeq_max_codeword; ++cw) {
        mod = cfg->modulation[cw];
        if ((Uchar)mod == 0) {
            mod = cfg->modulation[0]; /* 调制未配置，回退码字 0 */
        }
        qm = (Uint)mod; /* 调制枚举值即每符号比特数 */
        n = workspace->codeword_symbol_count[cw];
        workspace->soft_bit_offset[cw] = out;
        if (out + n * qm > workspace->soft_bit_tmp_capacity) {
            return ufeq_fail("demod", "buffer_small");
        }
        for (i = 0; i < n; ++i) {
            rho = ufeq_update_rho(workspace->codeword_sinr[cw][i], cfg);
            x = workspace->codeword_symbol[cw][i];
            dst = &workspace->soft_bit_tmp[out];
            switch (mod) {
            case ufeq_mod_pi_2_bpsk:
                ufeq_demod_pi2_bpsk(x, rho, i, dst, cfg->n_sb_shift, &workspace->stats);
                break;
            case ufeq_mod_qpsk:
                ufeq_demod_qpsk(x, rho, dst, cfg->n_sb_shift, &workspace->stats);
                break;
            case ufeq_mod_16qam:
                ufeq_demod_16qam(x, rho, d16, dst, cfg->n_sb_shift, &workspace->stats);
                break;
            case ufeq_mod_64qam:
                ufeq_demod_64qam(x, rho, d64, dst, cfg->n_sb_shift, &workspace->stats);
                break;
            case ufeq_mod_256qam:
                ufeq_demod_256qam(x, rho, d256, dst, cfg->n_sb_shift, &workspace->stats);
                break;
            default:
                return ufeq_fail("demod", "invalid_config");
            }
            out += qm; /* 推进软比特写入偏移 */
        }
        workspace->soft_bit_count[cw] = n * qm;
    }
    return ufeq_status_ok;
}
