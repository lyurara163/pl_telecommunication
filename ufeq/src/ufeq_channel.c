#include "ufeq_internal.h"   /* 内部头文件：配置/工作区/复数工具等声明 */

#include <string.h>          /* 提供 memcpy 等内存操作函数 */

/**
 * 在已排序的 DMRS 符号列表中，为数据符号 symb 找左右邻居，并算线性插值权重。
 *
 * 【返回】
 *   la, lb : 左右（或同一）DMRS 符号索引
 *   wa, wb : 对应权重，满足 H(symb) ≈ wa*H(la) + wb*H(lb)
 *
 * 【边界规则】
 *   - 仅 1 个 DMRS：直接复制该符号（wa=1, wb=0）
 *   - symb 落在首个 DMRS 之前：外推复制首端（wa=1）
 *   - symb 落在末个 DMRS 之后：外推复制末端（wa=1）
 *   - 落在 (la, lb) 之间：
 *       wa = (lb - symb) / (lb - la)
 *       wb = (symb - la) / (lb - la)
 */
static void ufeq_find_dmrs_neighbors(const ufeq_config_t *cfg, /* 配置：含 DMRS 符号列表 */
                                     Ushort symb,              /* 当前待插值的 OFDM 符号索引 */
                                     Ushort *la,               /* 输出：左侧（或唯一）DMRS 符号索引 */
                                     Ushort *lb,               /* 输出：右侧（或唯一）DMRS 符号索引 */
                                     float *wa,                /* 输出：左侧权重 */
                                     float *wb)                /* 输出：右侧权重 */
{
    Ushort i = 0;        /* 遍历 DMRS 列表的循环变量 */
    float den = 0.0;     /* 分母：两个 DMRS 符号间隔 (lb - la) */

    /* 情况1：slot 内只有 1 个 DMRS，无法插值，直接复制该符号 */
    if (cfg->n_rs == 1) {
        *la = cfg->dmrs_symbol[0]; /* 左右邻居都指向同一个 DMRS */
        *lb = cfg->dmrs_symbol[0];
        *wa = 1.0;                 /* 权重全给左侧（唯一）符号 */
        *wb = 0.0;                 /* 右侧权重为 0 */
        return;                    /* 处理完毕，提前返回 */
    }

    /* 情况2：目标符号在第一个 DMRS 之前（或正好等于），做首端外推：复制首 DMRS */
    if (symb <= cfg->dmrs_symbol[0]) {
        *la = cfg->dmrs_symbol[0]; /* 使用首个 DMRS */
        *lb = cfg->dmrs_symbol[0];
        *wa = 1.0;
        *wb = 0.0;
        return;
    }

    /* 情况3：目标符号在最后一个 DMRS 之后（或正好等于），做末端外推：复制末 DMRS */
    if (symb >= cfg->dmrs_symbol[cfg->n_rs - 1]) {
        *la = cfg->dmrs_symbol[cfg->n_rs - 1]; /* 使用最后一个 DMRS */
        *lb = cfg->dmrs_symbol[cfg->n_rs - 1];
        *wa = 1.0;
        *wb = 0.0;
        return;
    }

    /* 情况4：目标符号落在某两个相邻 DMRS 之间，做线性插值 */
    for (i = 0; i + 1 < cfg->n_rs; ++i) { /* 逐对检查相邻 DMRS：i 与 i+1 */
        if (symb >= cfg->dmrs_symbol[i] && symb <= cfg->dmrs_symbol[i + 1]) {
            den = (float)(cfg->dmrs_symbol[i + 1] - cfg->dmrs_symbol[i]); /* 两 DMRS 间距 */
            *la = cfg->dmrs_symbol[i];       /* 左邻居 */
            *lb = cfg->dmrs_symbol[i + 1];   /* 右邻居 */
            if (den <= 0.0) {                /* 异常：间距非正，退化为复制左侧 */
                *wa = 1.0;
                *wb = 0.0;
            } else {
                /* 线性插值权重：距右端越近，左权重越小；距左端越近，右权重越小 */
                *wa = ((float)(*lb) - (float)symb) / den; /* wa = (lb - symb) / (lb - la) */
                *wb = ((float)symb - (float)(*la)) / den; /* wb = (symb - la) / (lb - la) */
            }
            return; /* 已找到所在区间，返回 */
        }
    }

    /* 兜底：理论上不应走到这里（前面边界与区间已覆盖），复制首 DMRS */
    *la = cfg->dmrs_symbol[0];
    *lb = cfg->dmrs_symbol[0];
    *wa = 1.0;
    *wb = 0.0;
}

/**
 * 信道时域插值（Time-Domain Channel Interpolation）
 *
 * 【目的】
 *   DMRS 只在部分 OFDM 符号上有信道估计。数据符号需要用相邻 DMRS 做线性插值，
 *   得到全 slot 的 H，供 MMSE 均衡使用。
 *
 * 【前置条件】
 *   必须先完成 DMRS 频偏前校准。输入来自 workspace->channel_work。
 *
 * 【算法】
 *   对每个 OFDM 符号 l、每个 RE / 天线 / 层：
 *     H(l) = wa * H(la) + wb * H(lb)
 *
 * 【旁路】
 *   bypass_dmrs_time_filter=1：把 channel_work 原样拷到 channel_interp。
 *
 * 【输出】
 *   workspace->channel_interp ：全 slot 插值后的信道矩阵缓冲
 */
ufeq_status_t ufeq_channel_interpolate(const ufeq_request_t *request, /* 输入请求（含配置） */
                                       ufeq_workspace_t *workspace)   /* 工作区（含信道缓冲） */
{
    const ufeq_config_t *cfg = NULL; /* 指向请求中的配置结构 */
    Ushort symb = 0;                 /* 当前 OFDM 符号索引 */
    Ushort sc = 0;                   /* 当前子载波（PUSCH RE）线性索引 */
    Ushort rx = 0;                   /* 接收天线索引 */
    Ushort layer = 0;                /* 发送层索引 */
    Uint expected = 0;             /* 全 slot 信道缓冲应有的复数元素个数 */
    Ushort la = 0;                   /* 左邻居 DMRS 符号 */
    Ushort lb = 0;                   /* 右邻居 DMRS 符号 */
    float wa = 0.0;                  /* 左邻居插值权重 */
    float wb = 0.0;                  /* 右邻居插值权重 */
    Ushort rb = 0;                   /* 资源块（RB）索引 */
    Ushort re = 0;                   /* RB 内 RE（子载波）索引 */
    Uint ia = 0;                   /* 左邻居信道在缓冲中的线性下标 */
    Uint ib = 0;                   /* 右邻居信道在缓冲中的线性下标 */
    Uint io = 0;                   /* 输出（当前符号）信道线性下标 */

    /* 空指针检查：请求、工作区、配置缺一不可 */
    if (request == NULL || workspace == NULL || request->param.config == NULL) {
        return ufeq_fail("channel", "invalid_arg"); /* 返回参数非法错误 */
    }
    cfg = request->param.config; /* 取出配置指针，后续多次使用 */

    /* 计算全 slot 信道矩阵元素个数：符号数 × RB数 × 每RB子载波数 × 收天线数 × 层数 */
    expected = (Uint)cfg->n_symb_slot * cfg->n_rb * cfg->n_sc_rb * cfg->n_rx * cfg->n_layer;

    /* 输出缓冲必须已分配 */
    if (workspace->channel_interp == NULL) {
        return ufeq_fail("channel", "buffer_small"); /* 缓冲无效/过小 */
    }

    /* 旁路模式：不做时域插值，直接把校准后的 DMRS 信道整块拷到输出 */
    if (cfg->bypass_dmrs_time_filter) {
        memcpy(workspace->channel_interp, workspace->channel_work, expected * sizeof(ufeq_cfloat_t));
        return ufeq_status_ok; /* 旁路完成，成功返回 */
    }

    /* 对 slot 内每一个 OFDM 符号做时域插值 */
    for (symb = 0; symb < cfg->n_symb_slot; ++symb) {
        /* 为当前符号找到左右 DMRS 及插值权重 */
        ufeq_find_dmrs_neighbors(cfg, symb, &la, &lb, &wa, &wb);

        /* 遍历所有 PUSCH 子载波 */
        for (sc = 0; sc < cfg->m_sc_pusch; ++sc) {
            rb = (Ushort)(sc / cfg->n_sc_rb); /* 由线性子载波号换算 RB 索引 */
            re = (Ushort)(sc % cfg->n_sc_rb); /* 由线性子载波号换算 RB 内 RE 索引 */

            /* 遍历接收天线与发送层 */
            for (rx = 0; rx < cfg->n_rx; ++rx) {
                for (layer = 0; layer < cfg->n_layer; ++layer) {
                    /* 计算左/右邻居与输出位置在信道缓冲中的下标 */
                    ia = ufeq_channel_index(la, rb, re, rx, layer, cfg);    /* H(la) 下标 */
                    ib = ufeq_channel_index(lb, rb, re, rx, layer, cfg);    /* H(lb) 下标 */
                    io = ufeq_channel_index(symb, rb, re, rx, layer, cfg);  /* H(symb) 下标 */

                    /* H(symb) = wa * H(la) + wb * H(lb) ：复数加权后相加 */
                    workspace->channel_interp[io] = ufeq_cf_add(
                        ufeq_cf_scale(workspace->channel_work[ia], wa), /* wa * H(la) */
                        ufeq_cf_scale(workspace->channel_work[ib], wb)); /* wb * H(lb) */
                }
            }
        }
    }
    return ufeq_status_ok; /* 全 slot 插值完成，返回成功 */
}
