#include "ufeq_internal.h"   /* 内部头文件：定点类型与工具声明 */

#include <limits.h>          /* 提供 INT32_MAX / INT32_MIN 等极限常量 */

/**
 * 定点算术辅助：对称舍入右移、位宽饱和、复数定点乘法。
 * AGC 移位与最终均衡导出复用这些例程，保证舍入规则一致。
 */

/**
 * 32 位有符号对称舍入右移
 *
 * 【目的】
 *   将 64 位中间值按 shift 位右移，采用对称舍入（正负均加 0.5 LSB 再截断）。
 *
 * 【算法】
 *   bias = 1 << (shift - 1)
 *   正数：(value + bias) >> shift
 *   负数：-(((-value) + bias) >> shift)
 *
 * 【保护】
 *   shift=0 时直接饱和到 int32；shift≥63 时按符号返回 0 或 -1。
 */
int32_t ufeq_shift_right_round_s32(int64_t value, /* 待右移的 64 位值 */
                                   Uchar shift)    /* 右移位数 */
{
    int64_t bias = 0; /* 舍入偏置：0.5 LSB */

    if (shift == 0) {
        if (value > INT32_MAX) {
            return INT32_MAX;
        }
        if (value < INT32_MIN) {
            return INT32_MIN;
        }
        return (int32_t)value;
    }
    if (shift >= 63) {
        return (value >= 0) ? 0 : -1; /* 移位过大：正数→0，负数→-1 */
    }
    bias = (int64_t)1 << (shift - 1);
    if (value >= 0) {
        value = (value + bias) >> shift; /* 正数对称舍入右移 */
    } else {
        value = -(((-value) + bias) >> shift); /* 负数取绝对值舍入再还原符号 */
    }
    if (value > INT32_MAX) {
        return INT32_MAX;
    }
    if (value < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)value;
}

/**
 * 将 64 位值饱和到指定位宽的有符号整数（最高 bit_width 位有效）。
 *
 * 【参数】
 *   bit_width : 有效位宽 1~32；0 或 >32 时按 32 位处理
 */
int32_t ufeq_saturate_s32(int64_t value,    /* 待饱和的值 */
                          Uchar bit_width)  /* 目标位宽 */
{
    int64_t max_v = 0; /* 该位宽最大正值 */
    int64_t min_v = 0; /* 该位宽最小负值 */

    if (bit_width == 0 || bit_width > 32) {
        bit_width = 32;
    }
    if (bit_width == 32) {
        if (value > INT32_MAX) {
            return INT32_MAX;
        }
        if (value < INT32_MIN) {
            return INT32_MIN;
        }
        return (int32_t)value;
    }
    max_v = ((int64_t)1 << (bit_width - 1)) - 1;  /* 2^(w-1) - 1 */
    min_v = -((int64_t)1 << (bit_width - 1));     /* -2^(w-1) */
    if (value > max_v) {
        return (int32_t)max_v;
    }
    if (value < min_v) {
        return (int32_t)min_v;
    }
    return (int32_t)value;
}

/**
 * 将 32 位值饱和到 int16 范围 [-32768, 32767]。
 */
int16_t ufeq_saturate_s16(int32_t value) /* 待饱和的 32 位值 */
{
    if (value > 32767) {
        return 32767;
    }
    if (value < -32768) {
        return -32768;
    }
    return (int16_t)value;
}

/**
 * 将 32 位值对称饱和到 int8 范围 [-127, 127]（保留 -128 为保护值）。
 */
int8_t ufeq_saturate_s8_sym(int32_t value) /* 待饱和的 32 位值 */
{
    if (value > 127) {
        return 127;
    }
    if (value < -127) {
        return -127;
    }
    return (int8_t)value;
}

/**
 * 定点复数乘法：(a * b) 再右移 out_shift 位并舍入。
 *
 * 【算法】
 *   Re = (a.re*b.re - a.im*b.im) >> out_shift
 *   Im = (a.re*b.im + a.im*b.re) >> out_shift
 */
ufeq_cint32_t ufeq_complex_mul_q(ufeq_cint32_t a,    /* 乘数 A（定点复数） */
                                 ufeq_cint32_t b,    /* 乘数 B（定点复数） */
                                 Uchar out_shift)    /* 输出右移位数（定标还原） */
{
    int64_t re = 0;              /* 实部 64 位中间积 */
    int64_t im = 0;              /* 虚部 64 位中间积 */
    ufeq_cint32_t out = {0};     /* 输出定点复数 */

    re = (int64_t)a.re * (int64_t)b.re - (int64_t)a.im * (int64_t)b.im;
    im = (int64_t)a.re * (int64_t)b.im + (int64_t)a.im * (int64_t)b.re;
    out.re = ufeq_shift_right_round_s32(re, out_shift);
    out.im = ufeq_shift_right_round_s32(im, out_shift);
    return out;
}
