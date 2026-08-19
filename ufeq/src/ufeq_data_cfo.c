#include "ufeq_internal.h"   /* 内部头文件：配置/工作区/复数工具等声明 */

#include <math.h>            /* 提供 sqrtf 等数学函数（本文件间接依赖 ops） */

/**
 * 数据符号残余频偏估计与补偿（Data CFO / Residual Phase）
 *
 * 【目的】
 *   经过 DMRS 前校准、MMSE、后频偏之后，均衡星座仍可能存在整体旋转。
 *   本函数用数据符号自身估计该公共相位并补偿，改善后续解调。
 *
 * 【适用前提】
 *   参考实现按 QPSK 四象限质心设计。高阶调制时精度会下降。
 *
 * 【算法步骤】
 *   1) 幅度门限筛选：th_in < |x|^2 < th_out
 *   2) 按实/虚部符号分四象限累加质心
 *   3) phase_acc += c_q * conj(ref_q)，phase = atan2(Im, Re)
 *   4) 全体 equalized_work 乘 exp(-j*phase)
 *
 * 【旁路 / 退化】
 *   bypass_data_freq_offset=1，或无有效样本：phase=0
 *
 * 【输出】
 *   workspace->data_cfo_phase ：估计出的公共相位（弧度）
 *   workspace->equalized_work ：相位补偿后的均衡符号
 */
ufeq_status_t ufeq_data_cfo_process(const ufeq_request_t *request, /* 输入请求（含门限配置） */
                                      ufeq_workspace_t *workspace)   /* 工作区（读写 equalized_work） */
{
    const ufeq_config_t *cfg = NULL;       /* 指向配置结构 */
    ufeq_cfloat_t centroid[4] = {0};       /* 四象限样本累加和（质心分子） */
    ufeq_cfloat_t ref[4] = {0};            /* QPSK 四象限理想参考点 */
    Uint count[4] = {0};                   /* 各象限有效样本计数 */
    Uint i = 0;                          /* 遍历 equalized_work 的循环变量 */
    Uint n = 0;                          /* equalized_work 元素总数 */
    ufeq_cfloat_t phase_acc = {0};         /* 跨象限相位相关累加器 */
    float phase = 0.0;                     /* 估计出的公共相位（弧度） */
    float s = 0.0;                         /* sin(-phase) */
    float c = 0.0;                         /* cos(-phase) */
    ufeq_cfloat_t rot = {0};               /* 补偿旋转因子 exp(-j*phase) */
    Ushort q = 0;                          /* 当前样本所属象限索引 0~3 */
    ufeq_cfloat_t x = {0};                 /* 当前均衡符号 */
    float mag2 = 0.0;                      /* |x|^2，用于幅度门限筛选 */
    ufeq_cfloat_t c_mean = {0};            /* 某象限质心均值 */

    if (request == NULL || workspace == NULL || request->param.config == NULL) {
        return ufeq_fail("data_cfo", "invalid_arg");
    }
    cfg = request->param.config;
    if (cfg->bypass_data_freq_offset) {
        workspace->data_cfo_phase = 0.0;
        return ufeq_status_ok; /* 旁路：不做数据频偏估计 */
    }

    n = workspace->equalized_count;
    /* QPSK 四象限理想参考：(+1,+1), (-1,+1), (-1,-1), (+1,-1) */
    ref[0] = ufeq_cf(1.0, 1.0);
    ref[1] = ufeq_cf(-1.0, 1.0);
    ref[2] = ufeq_cf(-1.0, -1.0);
    ref[3] = ufeq_cf(1.0, -1.0);

    /* ---- 阶段 1：幅度门限筛选 + 四象限质心累加 ---- */
    for (i = 0; i < n; ++i) {
        x = workspace->equalized_work[i];
        mag2 = ufeq_cf_abs2(x);
        if (mag2 <= cfg->th_in || mag2 >= cfg->th_out) {
            continue; /* 幅度过小（噪声）或过大（异常），跳过 */
        }
        /* 按实/虚部符号判定 QPSK 象限 */
        if (x.re >= 0.0 && x.im >= 0.0) {
            q = 0; /* 第一象限 */
        } else if (x.re < 0.0 && x.im >= 0.0) {
            q = 1; /* 第二象限 */
        } else if (x.re < 0.0 && x.im < 0.0) {
            q = 2; /* 第三象限 */
        } else {
            q = 3; /* 第四象限 */
        }
        centroid[q] = ufeq_cf_add(centroid[q], x); /* 累加该象限样本 */
        count[q]++;
    }

    /* ---- 阶段 2：各象限质心与参考点做相关，累加公共相位 ---- */
    for (q = 0; q < 4; ++q) {
        if (count[q] > 0) {
            c_mean = ufeq_cf_scale(centroid[q], 1.0 / (float)count[q]); /* 质心均值 */
            /* phase_acc += c_mean * conj(ref_q)：估计相对理想点的旋转 */
            phase_acc = ufeq_cf_add(phase_acc, ufeq_cf_mul(c_mean, ufeq_cf_conj(ref[q])));
        }
    }

    /* 无有效样本：相位置 0，计 no_cfo_sample 统计 */
    if (phase_acc.re == 0.0 && phase_acc.im == 0.0) {
        workspace->data_cfo_phase = 0.0;
        workspace->stats.no_cfo_sample_count++;
        return ufeq_status_ok;
    }

    /* ---- 阶段 3：提取公共相位并对全体符号做补偿旋转 ---- */
    phase = workspace->ops->atan2_f32(phase_acc.im, phase_acc.re);
    workspace->data_cfo_phase = phase;
    workspace->ops->sin_cos_f32(-phase, &s, &c); /* exp(-j*phase) */
    rot = ufeq_cf(c, s);
    for (i = 0; i < n; ++i) {
        workspace->equalized_work[i] = ufeq_cf_mul(workspace->equalized_work[i], rot);
    }
    return ufeq_status_ok;
}
