#include "ufeq_internal.h"   /* 内部头：平台算子类型、复数工具、矩阵求逆 ref */

#include <math.h>            /* sinf/cosf/sqrtf/atan2f 等 libm 函数 */

#ifndef M_PI
#define M_PI 3.14159265358979323846  /* 圆周率，部分平台 math.h 未定义 */
#endif

/**
 * PC 参考平台算子实现
 *
 * 【职责】
 *   提供基于 libm 的 atan2/sin/cos/倒数，以及纯 C 的 IDFT；
 *   矩阵 Hermitian 求逆复用 ufeq_matrix_inverse_hermitian_ref。
 *
 * 【用途】
 *   默认注入 workspace->ops；DSP 集成时可替换为 ufeq_platform_dsp_ops()
 *   或自定义 ufeq_platform_ops_t，仅重写热点算子。
 */

/** 参考 atan2：直接调用 libm atan2f。 */
static float ufeq_ref_atan2(float y, float x)
{
    return atan2f(y, x);
}

/** 参考 sin/cos：同一 phase 一次算正弦与余弦。 */
static void ufeq_ref_sin_cos(float phase, float *sin_v, float *cos_v)
{
    *sin_v = sinf(phase);
    *cos_v = cosf(phase);
}

/**
 * 参考倒数：分母绝对值小于 ufeq_diag_protect 时钳位，避免除零。
 */
static float ufeq_ref_reciprocal(float x)
{
    if (fabsf(x) < ufeq_diag_protect) { /* 分母过小，按符号钳位到保护值 */
        x = (x >= 0.0) ? ufeq_diag_protect : -ufeq_diag_protect;
    }
    return 1.0 / x;
}

/**
 * 参考 IDFT（逆离散傅里叶变换）
 *
 * 【支持长度】
 *   length 须可分解为 2^a2 * 3^a3 * 5^a5（与 NR DFT 尺寸一致）。
 *
 * 【算法】
 *   直接 DFT 公式 O(N^2)，正确性优先；生产 DSP 应替换为 FFT 内核。
 *
 * 【归一化】
 *   输出乘以 1/sqrt(N)，与 DFT-s-OFDM 预编码归一化约定对齐。
 */
static ufeq_status_t ufeq_ref_idft(const ufeq_cfloat_t *in, ufeq_cfloat_t *out, Ushort length)
{
    Ushort k = 0;           /* 输出频域/时域 bin 索引 */
    Ushort i = 0;           /* 输入求和索引 */
    float inv_sqrt = 0.0;   /* 1/sqrt(length) 归一化因子 */
    Ushort m = 0;           /* 长度分解剩余因子 */
    Ushort a2 = 0;          /* 因子 2 的个数 */
    Ushort a3 = 0;          /* 因子 3 的个数 */
    Ushort a5 = 0;          /* 因子 5 的个数 */
    ufeq_cfloat_t sum = {0}; /* 单 bin 累加和 */
    float ang = 0.0;        /* 旋转因子角度 */
    ufeq_cfloat_t tw = {0}; /* 旋转因子 e^{-j*ang} */

    /* 参数检查：输入/输出指针非空，长度正 */
    if (in == NULL || out == NULL || length == 0) {
        return ufeq_fail("platform", "invalid_arg");
    }

    /* 分解 length = 2^a2 * 3^a3 * 5^a5，不支持则报错 */
    m = length;
    while ((m % 2) == 0) { /* 提取因子 2 */
        m = (Ushort)(m / 2);
        ++a2;
    }
    while ((m % 3) == 0) { /* 提取因子 3 */
        m = (Ushort)(m / 3);
        ++a3;
    }
    while ((m % 5) == 0) { /* 提取因子 5 */
        m = (Ushort)(m / 5);
        ++a5;
    }
    if (m != 1) { /* 剩余因子不为 1，长度不在支持集合内 */
        return ufeq_fail("platform", "unsupported_length");
    }

    inv_sqrt = 1.0 / sqrtf((float)length); /* 预计算归一化系数 */

    /* 对每个输出 bin k 做完整 DFT 求和 */
    for (k = 0; k < length; ++k) {
        sum = ufeq_cf(0.0, 0.0); /* 清零当前 bin 累加器 */
        for (i = 0; i < length; ++i) {
            /* IDFT 核：ang = -2π*i*k/N */
            ang = -2.0 * (float)M_PI * (float)i * (float)k / (float)length;
            tw = ufeq_cf(cosf(ang), sinf(ang)); /* W_N^{-ik} */
            /* sum += conj(in[i]) * W */
            sum = ufeq_cf_add(sum, ufeq_cf_mul(ufeq_cf_conj(in[i]), tw));
        }
        /* out[k] = conj(sum) / sqrt(N) */
        out[k] = ufeq_cf_conj(ufeq_cf_scale(sum, inv_sqrt));
    }
    return ufeq_status_ok;
}

/** 参考平台算子表静态实例，函数指针指向本文件 static 实现 + 公共矩阵求逆 ref。 */
static const ufeq_platform_ops_t g_ufeq_platform_ref = {
    ufeq_matrix_inverse_hermitian_ref, /* Hermitian 矩阵求逆 */
    ufeq_ref_idft,                     /* IDFT */
    ufeq_ref_atan2,                    /* atan2 */
    ufeq_ref_sin_cos,                  /* sin/cos */
    ufeq_ref_reciprocal                /* 保护倒数 */
};

/**
 * 返回参考平台算子表指针（进程内单例，只读）。
 * workspace 未 set_platform 时默认使用此表。
 */
const ufeq_platform_ops_t *ufeq_platform_ref_ops(void)
{
    return &g_ufeq_platform_ref;
}
