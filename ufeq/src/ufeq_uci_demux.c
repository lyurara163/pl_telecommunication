#include "ufeq_internal.h"   /* 内部头文件：配置/工作区/状态码等 */

#include <stdlib.h>          /* qsort 排序 */
#include <string.h>          /* memset 等内存操作 */

/**
 * UCI 解复用（PUSCH 上复用的 ACK / CSI 与数据分离）
 *
 * 【背景】
 *   PUSCH 上部分 RE 承载 UCI（ACK、CSI1、CSI2），部分承载 SCH 数据。
 *   用 pusch_re_flag 标记每个 RE 的类型；解复用分两阶段：
 *
 *   demux1（解调前，ufeq_uci_demux_first）
 *     - 按 flag 收集 ACK/CSI 占位 RE 的索引
 *     - 把对应均衡符号/SINR 拷到 result->uci_syms / uci_sinr，供 UCI 专用解调
 *     - 在 valid_re_flag 中标记这些 UCI RE
 *
 *   demux2（解扰后，ufeq_uci_demux_second）
 *     - 按同一套 flag，从解扰后的软比特流拆出：
 *         flag=0      -> SCH 数据软比特
 *         flag=1/-1/-2 -> ACK 软比特（-1/-2 还有打孔副作用）
 *         flag=2      -> CSI1
 *         flag=3      -> CSI2
 *
 * 【flag 约定（与上层配置一致）】
 *   -2 : ACK 与 CSI2 重叠打孔类
 *   -1 : ACK 打孔（SCH 对应位置补 0）
 *    0 : 纯 SCH 数据
 *    1 : ACK
 *    2 : CSI1
 *    3 : CSI2
 */

/**
 * Uint 升序比较函数，供 qsort 使用
 */
static int ufeq_u32_cmp(const void *a, const void *b)
{
    Uint va = 0;
    Uint vb = 0;
    va = *(const Uint *)a;
    vb = *(const Uint *)b;
    if (va < vb) {
        return -1;
    }
    if (va > vb) {
        return 1;
    }
    return 0;
}

/**
 * 按指定 flag 值收集 RE 线性索引
 *
 * 【输入/输出】
 *   flags     : RE 标志数组
 *   count     : RE 总数
 *   want      : 目标 flag 值
 *   out       : 输出索引数组
 *   out_count : 输出匹配个数
 */
static void ufeq_collect_flags(const int8_t *flags, /* RE 标志数组 */
                               Uint count,           /* RE 总数 */
                               int8_t want,          /* 目标 flag */
                               Uint *out,            /* 输出索引缓冲 */
                               Uint *out_count)      /* 输出匹配数 */
{
    Uint i = 0;
    Uint n = 0; /* 已收集个数 */
    for (i = 0; i < count; ++i) {
        if (flags[i] == want) {
            out[n++] = i;
        }
    }
    *out_count = n;
}

/**
 * demux1：解调前收集 UCI 符号
 *
 * 【步骤】
 *   1) 按 flag 分类收集索引（-2/-1/1/2/3），排序后合并到 ack/csi1/csi2 列表
 *   2) 从 codeword_symbol[0] 取出对应符号写入 result->uci_syms
 *   3) SINR：默认同 RE 的 codeword_sinr；若 pusch_uci_qpsk_sinr1_flag，则用码字平均 SINR
 *   4) 在 valid_re_flag 标记这些 UCI RE，供后续逻辑使用
 */
ufeq_status_t ufeq_uci_demux_first(const ufeq_request_t *request, /* 输入请求（含 pusch_re_flag） */
                                   ufeq_workspace_t *workspace,   /* 工作区（码字符号/SINR） */
                                   ufeq_result_t *result)         /* 输出 UCI 符号与 SINR */
{
    const ufeq_config_t *cfg = NULL;
    Uint re_count = 0;        /* PUSCH RE 总数 */
    Uint m2[4096] = {0};      /* flag=-2 索引（ACK+CSI2 重叠） */
    Uint m1[4096] = {0};      /* flag=-1 索引（ACK 打孔） */
    Uint f1[4096] = {0};      /* flag=1  索引（纯 ACK） */
    Uint f2[4096] = {0};      /* flag=2  索引（CSI1） */
    Uint f3[4096] = {0};      /* flag=3  索引（CSI2） */
    Uint ack[8192] = {0};     /* 合并后的 ACK RE 索引 */
    Uint csi1[4096] = {0};   /* 合并后的 CSI1 RE 索引 */
    Uint csi2[8192] = {0};    /* 合并后的 CSI2 RE 索引 */
    Uint n_m2 = 0;            /* 各 flag 收集计数 */
    Uint n_m1 = 0;
    Uint n_f1 = 0;
    Uint n_f2 = 0;
    Uint n_f3 = 0;
    Uint n_ack = 0;           /* 合并后 ACK/CSI 计数 */
    Uint n_csi1 = 0;
    Uint n_csi2 = 0;
    Uint i = 0;
    Uint out = 0;             /* uci_syms 写入偏移 */
    float mean_rho = 0.0;     /* 码字 0 平均 SINR（QPSK UCI 模式） */
    Uint mean_count = 0;
    Uint idx = 0;             /* 当前 RE 线性索引 */

    if (request == NULL || workspace == NULL || result == NULL || request->param.config == NULL) {
        return ufeq_fail("uci_demux", "invalid_arg");
    }
    cfg = request->param.config;
    if (cfg->bypass_demux_1) {
        result->uci_syms_count = 0;
        result->uci_sinr_count = 0;
        return ufeq_status_ok; /* 旁路 demux1 */
    }
    if (request->data.pusch_re_flag == NULL) {
        return ufeq_fail("uci_demux", "invalid_arg");
    }

    re_count = (Uint)request->data.pusch_re_flag_count;
    if (re_count > 4096) {
        return ufeq_fail("uci_demux", "out_of_range");
    }
    if (workspace->valid_re_flag != NULL && re_count > workspace->valid_re_flag_capacity) {
        return ufeq_fail("uci_demux", "buffer_small");
    }

    /* 按 flag 分类收集 RE 索引 */
    ufeq_collect_flags(request->data.pusch_re_flag, re_count, -2, m2, &n_m2);
    ufeq_collect_flags(request->data.pusch_re_flag, re_count, -1, m1, &n_m1);
    ufeq_collect_flags(request->data.pusch_re_flag, re_count, 1, f1, &n_f1);
    ufeq_collect_flags(request->data.pusch_re_flag, re_count, 2, f2, &n_f2);
    ufeq_collect_flags(request->data.pusch_re_flag, re_count, 3, f3, &n_f3);

    /* 各分类内部排序，便于合并去重 */
    qsort(m2, n_m2, sizeof(Uint), ufeq_u32_cmp);
    qsort(m1, n_m1, sizeof(Uint), ufeq_u32_cmp);
    qsort(f1, n_f1, sizeof(Uint), ufeq_u32_cmp);
    qsort(f2, n_f2, sizeof(Uint), ufeq_u32_cmp);
    qsort(f3, n_f3, sizeof(Uint), ufeq_u32_cmp);

    /* 合并到 ack / csi1 / csi2 列表 */
    for (i = 0; i < n_m2; ++i) {
        ack[n_ack++] = m2[i];   /* -2：同时计入 ACK 与 CSI2 */
        csi2[n_csi2++] = m2[i];
    }
    for (i = 0; i < n_m1; ++i) {
        ack[n_ack++] = m1[i];   /* -1：ACK 打孔 */
    }
    for (i = 0; i < n_f1; ++i) {
        ack[n_ack++] = f1[i];   /* 1：纯 ACK */
    }
    for (i = 0; i < n_f2; ++i) {
        csi1[n_csi1++] = f2[i]; /* 2：CSI1 */
    }
    for (i = 0; i < n_f3; ++i) {
        csi2[n_csi2++] = f3[i]; /* 3：CSI2 */
    }
    qsort(ack, n_ack, sizeof(Uint), ufeq_u32_cmp);
    qsort(csi1, n_csi1, sizeof(Uint), ufeq_u32_cmp);
    qsort(csi2, n_csi2, sizeof(Uint), ufeq_u32_cmp);

    /* 输出缓冲容量检查 */
    if (result->uci_syms_capacity < (n_ack + n_csi1 + n_csi2)
        || result->uci_sinr_capacity < (n_ack + n_csi1 + n_csi2)) {
        return ufeq_fail("uci_demux", "buffer_small");
    }

    /* QPSK UCI 模式：使用码字 0 平均 SINR 替代逐 RE SINR */
    if (cfg->pusch_uci_qpsk_sinr1_flag != 0) {
        for (i = 0; i < workspace->codeword_symbol_count[0]; ++i) {
            mean_rho += workspace->codeword_sinr[0][i];
            mean_count++;
        }
        if (mean_count > 0) {
            mean_rho /= (float)mean_count;
        }
    }

    /* 依次输出 ACK → CSI1 → CSI2 符号与 SINR */
    for (i = 0; i < n_ack; ++i) {
        idx = ack[i];
        if (idx >= workspace->codeword_symbol_count[0]) {
            continue; /* 索引越界保护 */
        }
        result->uci_syms[out] = workspace->codeword_symbol[0][idx];
        result->uci_sinr[out] =
            (cfg->pusch_uci_qpsk_sinr1_flag != 0) ? mean_rho : workspace->codeword_sinr[0][idx];
        ++out;
    }
    for (i = 0; i < n_csi1; ++i) {
        idx = csi1[i];
        if (idx >= workspace->codeword_symbol_count[0]) {
            continue;
        }
        result->uci_syms[out] = workspace->codeword_symbol[0][idx];
        result->uci_sinr[out] =
            (cfg->pusch_uci_qpsk_sinr1_flag != 0) ? mean_rho : workspace->codeword_sinr[0][idx];
        ++out;
    }
    for (i = 0; i < n_csi2; ++i) {
        idx = csi2[i];
        if (idx >= workspace->codeword_symbol_count[0]) {
            continue;
        }
        result->uci_syms[out] = workspace->codeword_symbol[0][idx];
        result->uci_sinr[out] =
            (cfg->pusch_uci_qpsk_sinr1_flag != 0) ? mean_rho : workspace->codeword_sinr[0][idx];
        ++out;
    }
    result->uci_syms_count = out;
    result->uci_sinr_count = out;

    /* 在 valid_re_flag 标记所有 UCI RE */
    if (workspace->valid_re_flag != NULL) {
        memset(workspace->valid_re_flag, 0, re_count);
        for (i = 0; i < n_ack; ++i) {
            workspace->valid_re_flag[ack[i]] = 1;
        }
        for (i = 0; i < n_csi1; ++i) {
            workspace->valid_re_flag[csi1[i]] = 1;
        }
        for (i = 0; i < n_csi2; ++i) {
            workspace->valid_re_flag[csi2[i]] = 1;
        }
    }
    return ufeq_status_ok;
}

/**
 * demux2：解扰后按 flag 拆软比特到 SCH / ACK / CSI 缓冲区
 *
 * 【要点】
 *   每个 RE 对应 qm=modulation[0] 个软比特。
 *   flag=-1：ACK 取走软比特后，SCH 流对应位置补 0（打孔占位）。
 *   flag=-2：ACK 取走后，CSI2 流对应位置补 0。
 *   可选输出 index_*（RE 索引）与 scram_*（对应扰码比特）。
 */
ufeq_status_t ufeq_uci_demux_second(const ufeq_request_t *request, /* 输入请求（含 pusch_re_flag） */
                                    ufeq_workspace_t *workspace,   /* 工作区（soft_bit_tmp / scram_tmp） */
                                    ufeq_result_t *result)         /* 输出 SCH/ACK/CSI 软比特 */
{
    const ufeq_config_t *cfg = NULL;
    Uint re_count = 0;   /* PUSCH RE 总数 */
    Uint i = 0;          /* RE 遍历索引 */
    Uint qm = 0;         /* 每 RE 软比特数 = 调制阶数 */
    Uint sch_n = 0;      /* SCH RE 计数（预留统计） */
    Uint ack_n = 0;      /* ACK RE 计数 */
    Uint csi1_n = 0;     /* CSI1 RE 计数 */
    Uint csi2_n = 0;     /* CSI2 RE 计数 */
    Uint soft_idx = 0;   /* soft_bit_tmp 中当前 RE 的起始偏移 */
    int8_t flag = 0;     /* 当前 RE 的 UCI 标志 */
    Uint b = 0;          /* 软比特内循环索引 */
    Uint bb = 0;         /* 打孔补零循环索引 */

    if (request == NULL || workspace == NULL || result == NULL || request->param.config == NULL) {
        return ufeq_fail("uci_demux", "invalid_arg");
    }
    cfg = request->param.config;
    if (cfg->bypass_demux_2) {
        return ufeq_status_ok; /* 旁路 demux2 */
    }
    if (request->data.pusch_re_flag == NULL || workspace->soft_bit_tmp == NULL) {
        return ufeq_fail("uci_demux", "invalid_arg");
    }

    qm = (Uint)cfg->modulation[0]; /* 按码字 0 调制阶数确定每 RE 比特数 */
    re_count = (Uint)request->data.pusch_re_flag_count;

    /* 清零各输出计数 */
    result->sch_count = 0;
    result->ack_count = 0;
    result->csi1_count = 0;
    result->csi2_count = 0;

    /* 逐 RE 按 flag 分流软比特 */
    for (i = 0; i < re_count; ++i) {
        flag = request->data.pusch_re_flag[i];
        soft_idx = i * qm; /* 第 i 个 RE 在软比特流中的起始位置 */
        if (soft_idx + qm > workspace->soft_bit_tmp_capacity) {
            return ufeq_fail("uci_demux", "buffer_small");
        }
        if (flag == 0) {
            /* 纯 SCH 数据 RE */
            if (result->sch_count + qm > result->sch_capacity) {
                return ufeq_fail("uci_demux", "buffer_small");
            }
            for (b = 0; b < qm; ++b) {
                result->sch_soft_bit[result->sch_count++] = workspace->soft_bit_tmp[soft_idx + b];
            }
            sch_n++;
        } else if (flag == 1 || flag == -1 || flag == -2) {
            /* ACK 类 RE（含打孔 flag=-1/-2） */
            if (result->ack_count + qm > result->ack_capacity) {
                return ufeq_fail("uci_demux", "buffer_small");
            }
            for (b = 0; b < qm; ++b) {
                result->ack_soft_bit[result->ack_count++] = workspace->soft_bit_tmp[soft_idx + b];
            }
            if (result->index_ack != NULL) {
                result->index_ack[ack_n] = i; /* 记录 ACK RE 索引 */
            }
            if (result->scram_ack != NULL && workspace->scram_tmp != NULL) {
                for (b = 0; b < qm; ++b) {
                    result->scram_ack[ack_n * qm + b] = workspace->scram_tmp[soft_idx + b];
                }
            }
            if (flag == -1) {
                /* ACK 打孔：SCH 流对应位置补 0 */
                if (result->sch_soft_bit == NULL || result->sch_count + qm > result->sch_capacity) {
                    return ufeq_fail("uci_demux", "buffer_small");
                }
                for (bb = 0; bb < qm; ++bb) {
                    result->sch_soft_bit[result->sch_count++] = 0;
                }
            }
            if (flag == -2) {
                /* ACK+CSI2 重叠打孔：CSI2 流对应位置补 0 */
                if (result->csi2_soft_bit == NULL || result->csi2_count + qm > result->csi2_capacity) {
                    return ufeq_fail("uci_demux", "buffer_small");
                }
                for (bb = 0; bb < qm; ++bb) {
                    result->csi2_soft_bit[result->csi2_count++] = 0;
                }
            }
            ack_n++;
        } else if (flag == 2) {
            /* CSI1 RE */
            if (result->csi1_count + qm > result->csi1_capacity) {
                return ufeq_fail("uci_demux", "buffer_small");
            }
            for (b = 0; b < qm; ++b) {
                result->csi1_soft_bit[result->csi1_count++] = workspace->soft_bit_tmp[soft_idx + b];
            }
            if (result->index_csi1 != NULL) {
                result->index_csi1[csi1_n] = i;
            }
            if (result->scram_csi1 != NULL && workspace->scram_tmp != NULL) {
                for (b = 0; b < qm; ++b) {
                    result->scram_csi1[csi1_n * qm + b] = workspace->scram_tmp[soft_idx + b];
                }
            }
            csi1_n++;
        } else if (flag == 3) {
            /* CSI2 RE */
            if (result->csi2_count + qm > result->csi2_capacity) {
                return ufeq_fail("uci_demux", "buffer_small");
            }
            for (b = 0; b < qm; ++b) {
                result->csi2_soft_bit[result->csi2_count++] = workspace->soft_bit_tmp[soft_idx + b];
            }
            if (result->index_csi2 != NULL) {
                result->index_csi2[csi2_n] = i;
            }
            if (result->scram_csi2 != NULL && workspace->scram_tmp != NULL) {
                for (b = 0; b < qm; ++b) {
                    result->scram_csi2[csi2_n * qm + b] = workspace->scram_tmp[soft_idx + b];
                }
            }
            csi2_n++;
        }
    }
    (void)sch_n;
    return ufeq_status_ok;
}
