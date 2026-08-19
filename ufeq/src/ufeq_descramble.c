#include "ufeq_internal.h"   /* 内部头文件：配置/工作区/工具函数等声明 */

#include <string.h>          /* 提供 memcpy 等内存操作函数 */

/**
 * 生成 Gold 伪随机序列（3GPP 38.211 §5.2.1）
 *
 * 【目的】
 *   按 c_init 初始化 x1/x2 两个 31-bit LFSR，跳过前 Nc=1600 位后，
 *   输出 c(n) = x1(n) ⊕ x2(n) 共 length 位，供软比特解扰使用。
 *
 * 【输入】
 *   c_init : Gold 序列初始化值（低 31 位有效）
 *   length : 需生成的扰码比特数
 *
 * 【输出】
 *   out    : 长度为 length 的 0/1 扰码序列
 */
static void ufeq_gold_generate(Uint c_init,    /* Gold 序列初始化值 */
                               Uchar *out,     /* 输出扰码比特缓冲 */
                               Uint length)  /* 输出长度（比特数） */
{
    Uint x1 = 1;           /* LFSR1 状态，固定初值 1 */
    Uint x2 = 0;           /* LFSR2 状态，由 c_init 初始化 */
    Uint n = 0;          /* 循环计数 */
    Uint new_bit = 0;      /* 本次反馈新比特 */

    x2 = c_init & 0x7fffffff; /* c_init 取低 31 位作为 x2 初态 */
    /* 跳过前 Nc=1600 位（标准规定，不输出） */
    for (n = 0; n < 1600; ++n) {
        new_bit = ((x1 >> 3) ^ x1) & 1; /* x1 反馈多项式 */
        x1 = (x1 >> 1) | (new_bit << 30);
        new_bit = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 1; /* x2 反馈多项式 */
        x2 = (x2 >> 1) | (new_bit << 30);
    }
    /* 正式输出 c(n) = x1 ⊕ x2，并推进 LFSR */
    for (n = 0; n < length; ++n) {
        out[n] = (Uchar)((x1 ^ x2) & 1);
        new_bit = ((x1 >> 3) ^ x1) & 1;
        x1 = (x1 >> 1) | (new_bit << 30);
        new_bit = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 1;
        x2 = (x2 >> 1) | (new_bit << 30);
    }
}

/**
 * 判断 value 是否存在于索引数组 index[0..count-1] 中。
 *
 * 【返回】1 表示存在，0 表示不存在或 index 为空。
 */
static int ufeq_index_contains(const Uint *index, /* 索引数组 */
                               Uint count,       /* 数组长度 */
                               Uint value)         /* 待查找的值 */
{
    Uint k = 0;

    if (index == NULL) {
        return 0;
    }
    for (k = 0; k < count; ++k) {
        if (index[k] == value) {
            return 1;
        }
    }
    return 0;
}

/**
 * 按扰码序列 c(i) 对软比特取反（解扰）。
 *
 * 【算法】
 *   c(i)=1 时 soft[i] 取反；c(i)=0 时不变。
 *   支持 UCI≤2bit 的 φx/φy 占位规则（38.211 §6.3.1.1）：
 *     φx：该比特位置不解扰（跳过）
 *     φy：用 c(i-1) 代替 c(i) 决定是否取反
 *   -128 为保护值，取反前先改为 -127 避免溢出。
 *
 * 【输入】
 *   soft/scram : 软比特与扰码序列（等长 bit_len）
 *   phi_x/phi_y: UCI 占位索引表（apply_phi=1 时生效）
 */
static void ufeq_descramble_bits(int8_t *soft,          /* 待解扰软比特（原地修改） */
                                 const Uchar *scram,    /* 扰码序列 c(i) */
                                 Uint bit_len,        /* 比特长度 */
                                 const Uint *phi_x,     /* φx 占位索引表 */
                                 Uint phi_x_n,        /* φx 索引个数 */
                                 const Uint *phi_y,     /* φy 占位索引表 */
                                 Uint phi_y_n,        /* φy 索引个数 */
                                 Uchar apply_phi)       /* 是否启用 φx/φy 规则 */
{
    Uint i = 0;     /* 当前比特索引 */
    int8_t v = 0;     /* 临时软比特值 */
    Uchar c = 0;      /* 当前使用的扰码比特 */

    for (i = 0; i < bit_len; ++i) {
        if (apply_phi && ufeq_index_contains(phi_x, phi_x_n, (Uint)i)) {
            continue; /* φx 位置：不解扰，直接跳过 */
        }
        if (apply_phi && ufeq_index_contains(phi_y, phi_y_n, (Uint)i)) {
            c = (i > 0) ? scram[i - 1] : 0; /* φy 位置：用 c(i-1) 代替 c(i) */
        } else {
            c = scram[i];
        }
        if (c != 0) {
            v = soft[i];
            if (v == (int8_t)-128) {
                v = -127; /* 避免 -(-128) 溢出 */
            }
            soft[i] = (int8_t)(-v); /* c=1：软比特取反即解扰 */
        }
    }
}

/**
 * 软比特解扰主流程
 *
 * 【目的】
 *   按 3GPP 规则生成 Gold 扰码序列，对各码字软比特做解扰。
 *
 * 【算法】
 *   c_init = n_rnti·2^15 + n_id，各码字独立序列：c_init + codeword_index
 *   HARQ/CSI a_len≤2 且 request 提供 φx/φy 索引时，码字 0 按占位规则解扰。
 *
 * 【旁路】
 *   bypass_descramble=1 时直接返回。
 *
 * 【输出】
 *   workspace->soft_bit_tmp ：原地解扰后的软比特
 */
ufeq_status_t ufeq_descramble_process(const ufeq_request_t *request, /* 输入请求（含 φ 索引） */
                                      ufeq_workspace_t *workspace)   /* 工作区（软比特/扰码缓冲） */
{
    const ufeq_config_t *cfg = NULL; /* 指向配置结构 */
    Uint c_init = 0;                 /* Gold 序列初值 */
    Uchar cw = 0;                    /* 当前码字索引 */
    Uchar cw_count = 0;              /* 有效码字数 */
    Uint offset = 0;               /* 当前码字在 soft_bit_tmp 中的偏移 */
    Uint bit_count = 0;            /* 当前码字软比特数 */
    Uchar need_phi = 0;              /* 是否启用 φx/φy 占位规则 */

    if (request == NULL || workspace == NULL || request->param.config == NULL) {
        return ufeq_fail("descramble", "invalid_arg");
    }
    cfg = request->param.config;
    if (cfg->bypass_descramble) {
        return ufeq_status_ok; /* 旁路：不解扰 */
    }
    if (workspace->soft_bit_tmp == NULL || workspace->scram_tmp == NULL) {
        return ufeq_fail("descramble", "buffer_small");
    }

    /* 确定实际码字数：优先 effective，其次 config，兜底 1 */
    cw_count = workspace->effective_codeword_count;
    if (cw_count == 0) {
        cw_count = cfg->codeword_count;
    }
    if (cw_count == 0) {
        cw_count = 1;
    }

    /* 判断是否需要 UCI φx/φy 占位规则：a_len≤2 且提供了索引表 */
    need_phi = 0;
    if ((cfg->pusch_harq_a_len > 0 && cfg->pusch_harq_a_len <= 2) ||
        (cfg->pusch_csi1_a_len > 0 && cfg->pusch_csi1_a_len <= 2) ||
        (cfg->pusch_csi2_a_len > 0 && cfg->pusch_csi2_a_len <= 2)) {
        if ((request->data.phi_x_index != NULL && request->data.phi_x_count > 0) ||
            (request->data.phi_y_index != NULL && request->data.phi_y_count > 0)) {
            need_phi = 1;
        }
    }

    c_init = ((Uint)cfg->n_rnti << 15) + (Uint)cfg->n_id; /* 38.211 标准 c_init 公式 */

    for (cw = 0; cw < cw_count && cw < ufeq_max_codeword; ++cw) {
        offset = workspace->soft_bit_offset[cw];
        bit_count = workspace->soft_bit_count[cw];
        if (bit_count == 0) {
            continue; /* 该码字无软比特，跳过 */
        }
        if (offset + bit_count > workspace->soft_bit_tmp_capacity) {
            return ufeq_fail("descramble", "buffer_small");
        }
        /* 生成该码字对应的 Gold 扰码序列 */
        ufeq_gold_generate(c_init + (Uint)cw, &workspace->scram_tmp[offset], bit_count);
        /* UCI 占位仅作用于码字 0（与 38.211 / MATLAB 约定一致） */
        ufeq_descramble_bits(&workspace->soft_bit_tmp[offset],
                             &workspace->scram_tmp[offset],
                             bit_count,
                             request->data.phi_x_index,
                             request->data.phi_x_count,
                             request->data.phi_y_index,
                             request->data.phi_y_count,
                             (Uchar)(need_phi && cw == 0));
    }

    return ufeq_status_ok;
}
