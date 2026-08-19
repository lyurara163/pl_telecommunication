#include "ufeq_internal.h"   /* 内部头文件：配置/工作区/定点工具等声明 */

#include <string.h>          /* 提供 memcpy 等内存操作函数 */

/**
 * AGC 增益拉齐（Automatic Gain Control Align）
 *
 * 【目的】
 *   不同天线 / 符号上的 AGC 因子可能不同，直接进均衡会导致幅度不一致。
 *   本函数把同一批样本对齐到统一参考增益，便于后续 MMSE / SINR 比较。
 *
 * 【入口】
 *   参数：param->config / param->start_rb
 *   数据：data->sch_freq_data，布局同 gde_sch_freq_data[slot][ant][symb]
 *         每个 unit 含 freq_data[264*12] 与 agc（每 ant×symb 一个因子）
 *
 * 【算法】
 *   1) 在用到的 (rx, symb) unit.agc 中取最大值 g_ref
 *   2) 对每个 unit：g_diff = g_ref - unit.agc
 *   3) 对该 unit 内调度 RB 的每个 RE：定点 I/Q 对称舍入右移 g_diff，再转浮点写 rx_work
 *
 * 【保护】
 *   g_diff < 0  ：钳到 0，并计 saturation
 *   g_diff > 31 ：钳到 31，并更新 agc_max_shift
 *
 * 【旁路】
 *   bypass_agc=1：不做移位，仅定点->浮点拷贝（scale=1）。
 *
 * 【输出】
 *   workspace->rx_work ：增益对齐后的接收浮点缓冲
 */
ufeq_status_t ufeq_agc_align(const ufeq_param_t *param,       /* 调度参数（含 config / start_rb） */
                             const ufeq_data_t *data,         /* 输入频域数据与 AGC 因子 */
                             ufeq_workspace_t *workspace)     /* 工作区（输出 rx_work） */
{
    const ufeq_config_t *cfg = NULL;  /* 指向配置结构 */
    int16_t g_ref = 0;                /* 参考最大 AGC 因子（全 ant×symb 扫描得到） */
    Uint expected = 0;              /* rx_work 应有的复数元素个数 */
    Ushort symb = 0;                  /* 当前 OFDM 符号索引 */
    Ushort rb = 0;                    /* 当前 RB 索引（相对调度块） */
    Ushort re = 0;                    /* RB 内 RE（子载波）索引 */
    Ushort rx = 0;                    /* 接收天线索引 */
    Ushort start_rb = 0;              /* 调度起始 RB（绝对 RB 号偏移） */
    Uint idx = 0;                   /* rx_work 线性下标 */
    Uint freq_idx = 0;              /* sch_freq_data 频域数组线性下标 */
    int16_t g_diff = 0;               /* 相对参考的右移位数：g_ref - unit.agc */
    ufeq_cint32_t sample = {0};       /* 解包后的定点 I/Q 样本 */
    const ufeq_sch_freq_data_unit_t *unit = NULL; /* 当前 (rx, symb) 频域数据单元 */
    bool first = true;                /* 首次扫描 g_ref 的标志 */

    /* 空指针检查：参数、数据、工作区、配置缺一不可 */
    if (param == NULL || data == NULL || workspace == NULL || param->config == NULL) {
        return ufeq_fail("agc", "invalid_arg");
    }
    cfg = param->config;
    start_rb = param->start_rb;
    /* 计算接收缓冲元素个数：符号数 × RB数 × 每RB子载波数 × 收天线数 */
    expected = (Uint)cfg->n_symb_slot * cfg->n_rb * cfg->n_sc_rb * cfg->n_rx;

    if (data->sch_freq_data == NULL) {
        return ufeq_fail("agc", "invalid_arg");
    }
    /* 调度 RB 范围不得超出 sch_freq_data 支持的 RB 上限 */
    if ((Ushort)(start_rb + cfg->n_rb) > ufeq_sch_freq_max_rb) {
        return ufeq_fail("agc", "invalid_arg");
    }
    /* 天线数、符号数不得超过 sch_freq_data 布局上限 */
    if (cfg->n_rx > ufeq_sch_freq_max_ant || cfg->n_symb_slot > ufeq_sch_freq_max_symb) {
        return ufeq_fail("agc", "invalid_arg");
    }
    if (workspace->rx_work == NULL || workspace->rx_work_count < expected) {
        return ufeq_fail("agc", "buffer_small");
    }

    /* 旁路模式：不做 AGC 移位，仅定点解包转浮点（scale=1）写入 rx_work */
    if (cfg->bypass_agc) {
        for (symb = 0; symb < cfg->n_symb_slot; ++symb) {
            for (rb = 0; rb < cfg->n_rb; ++rb) {
                for (re = 0; re < cfg->n_sc_rb; ++re) {
                    for (rx = 0; rx < cfg->n_rx; ++rx) {
                        unit = ufeq_sch_freq_unit(data->sch_freq_data, rx, symb);
                        /* 绝对 RB 号 × 每RB子载波数 + RE 索引 → 频域数组下标 */
                        freq_idx = ((Uint)(start_rb + rb) * cfg->n_sc_rb) + re;
                        idx = ufeq_rx_index(symb, rb, re, rx, cfg);
                        sample = ufeq_unpack_freq_uint(unit->freq_data[freq_idx]);
                        workspace->rx_work[idx] = ufeq_cint32_to_cf(sample, 1.0);
                    }
                }
            }
        }
        workspace->rx_work_owns = true; /* 标记 rx_work 已由本模块填充 */
        return ufeq_status_ok;
    }

    /* ---- 第一次扫描：求参考最大因子 g_ref（遍历所有 ant×symb 的 unit.agc） ---- */
    for (rx = 0; rx < cfg->n_rx; ++rx) {
        for (symb = 0; symb < cfg->n_symb_slot; ++symb) {
            unit = ufeq_sch_freq_unit(data->sch_freq_data, rx, symb);
            if (first || (int16_t)unit->agc > g_ref) {
                g_ref = (int16_t)unit->agc; /* 取最大 AGC 作为统一参考 */
                first = false;
            }
        }
    }

    /* ---- 第二次扫描：按 g_diff 右移并对齐到浮点工作区 ---- */
    for (symb = 0; symb < cfg->n_symb_slot; ++symb) {
        for (rx = 0; rx < cfg->n_rx; ++rx) {
            unit = ufeq_sch_freq_unit(data->sch_freq_data, rx, symb);
            g_diff = (int16_t)(g_ref - (int16_t)unit->agc); /* 需右移位数 = 参考 - 当前 */
            if (g_diff < 0) {
                /* 当前 AGC 已大于参考，不应再左移，钳到 0 并计饱和 */
                g_diff = 0;
                workspace->stats.saturation_count++;
            }
            if (g_diff > 31) {
                /* 右移超过 31 位会丢失全部有效位，钳到 31 并记录最大移位 */
                if ((Uint)g_diff > workspace->stats.agc_max_shift) {
                    workspace->stats.agc_max_shift = (Uint)g_diff;
                }
                g_diff = 31;
                workspace->stats.saturation_count++;
            }
            for (rb = 0; rb < cfg->n_rb; ++rb) {
                for (re = 0; re < cfg->n_sc_rb; ++re) {
                    freq_idx = ((Uint)(start_rb + rb) * cfg->n_sc_rb) + re;
                    idx = ufeq_rx_index(symb, rb, re, rx, cfg);
                    sample = ufeq_unpack_freq_uint(unit->freq_data[freq_idx]);
                    if (g_diff > 0) {
                        /* 对称舍入右移 g_diff 位，使该 unit 幅度对齐到 g_ref */
                        sample.re = ufeq_shift_right_round_s32(sample.re, (Uchar)g_diff);
                        sample.im = ufeq_shift_right_round_s32(sample.im, (Uchar)g_diff);
                    }
                    workspace->rx_work[idx] = ufeq_cint32_to_cf(sample, 1.0);
                }
            }
        }
    }
    workspace->rx_work_owns = true;
    return ufeq_status_ok;
}
