#include "ufeq.h"              /* 对外 API 类型：DMRS 配置枚举等 */
#include "ufeq_internal.h"     /* 内部头文件：失败宏、常量上限等 */

#include <string.h>            /* 提供 memset 等内存操作函数 */

/**
 * PUSCH DMRS 端口参数与时域位置（3GPP TS 38.211 §6.4.1.1.3）
 *
 * 【端口表含义】
 *   每个端口给出：
 *     - CDM 组编号
 *     - 频域偏移 delta
 *     - 频域 OCC 权重 w_f[2]
 *     - 时域 OCC 权重 w_t[2]
 *   单符号配置下：Type1 仅端口 0~3 有效，Type2 仅端口 0~5 有效；
 *   双符号配置才开放更高端口（用到时域 OCC 的第二符号）。
 */

/* 表 6.4.1.1.3-1：配置类型 1，端口 0~7 */
static const ufeq_dmrs_port_param_t g_type1_ports[8] = {
    {0, 0, {1, 1}, {1, 1}},
    {0, 0, {1, -1}, {1, 1}},
    {1, 1, {1, 1}, {1, 1}},
    {1, 1, {1, -1}, {1, 1}},
    {0, 0, {1, 1}, {1, -1}},
    {0, 0, {1, -1}, {1, -1}},
    {1, 1, {1, 1}, {1, -1}},
    {1, 1, {1, -1}, {1, -1}}
};

/* 表 6.4.1.1.3-2：配置类型 2，端口 0~11 */
static const ufeq_dmrs_port_param_t g_type2_ports[12] = {
    {0, 0, {1, 1}, {1, 1}},
    {0, 0, {1, -1}, {1, 1}},
    {1, 2, {1, 1}, {1, 1}},
    {1, 2, {1, -1}, {1, 1}},
    {2, 4, {1, 1}, {1, 1}},
    {2, 4, {1, -1}, {1, 1}},
    {0, 0, {1, 1}, {1, -1}},
    {0, 0, {1, -1}, {1, -1}},
    {1, 2, {1, 1}, {1, -1}},
    {1, 2, {1, -1}, {1, -1}},
    {2, 4, {1, 1}, {1, -1}},
    {2, 4, {1, -1}, {1, -1}}
};

/**
 * 查询 DMRS 端口参数
 *
 * 【目的】
 *   按配置类型（Type1/Type2）、端口号、是否双符号，查表返回 CDM/OCC 参数。
 *
 * 【约束】
 *   单符号时 Type1 仅 port 0~3，Type2 仅 port 0~5 合法。
 *
 * 【输出】
 *   param：端口参数结构体
 */
ufeq_status_t ufeq_dmrs_get_port_param(ufeq_dmrs_config_type_t config_type, /* DMRS 配置类型 */
                                       Uchar port,                         /* DMRS 端口号 */
                                       Uchar dual_symbol,                  /* 是否双符号 DMRS */
                                       ufeq_dmrs_port_param_t *param)      /* 输出：端口参数 */
{
    if (param == NULL) {
        return ufeq_fail("dmrs", "invalid_arg");
    }
    if (config_type == ufeq_dmrs_config_type1) {
        /* Type1：单符号时 port>3 无效；双符号时 port 可达 7 */
        if (port > 7 || (dual_symbol == 0 && port > 3)) {
            return ufeq_fail("dmrs", "invalid_config");
        }
        *param = g_type1_ports[port];
        return ufeq_status_ok;
    }
    if (config_type == ufeq_dmrs_config_type2) {
        /* Type2：单符号时 port>5 无效；双符号时 port 可达 11 */
        if (port > 11 || (dual_symbol == 0 && port > 5)) {
            return ufeq_fail("dmrs", "invalid_config");
        }
        *param = g_type2_ports[port];
        return ufeq_status_ok;
    }
    return ufeq_fail("dmrs", "invalid_config");
}

/**
 * 向 DMRS 符号位置集合插入索引：去重并保持升序。
 *
 * 【输入】
 *   set  : 符号集合（symbols[] + count）
 *   symb : 待插入的 OFDM 符号索引
 */
static void ufeq_push_symbol(ufeq_dmrs_symbol_set_t *set, /* 符号集合 */
                             Uchar symb)                   /* 待插入符号索引 */
{
    Uchar i = 0; /* 遍历已有符号 */
    Uchar j = 0; /* 插入位置后移循环变量 */

    for (i = 0; i < set->count; ++i) {
        if (set->symbols[i] == symb) {
            return; /* 已存在，去重直接返回 */
        }
    }
    if (set->count < ufeq_max_symb_slot) {
        /* 从尾部向前找插入点，保持升序 */
        for (j = set->count; j > 0 && set->symbols[j - 1] > symb; --j) {
            set->symbols[j] = set->symbols[j - 1];
        }
        set->symbols[j] = symb;
        ++set->count;
    }
}

/**
 * 查 DMRS 时域位置
 *
 * 【无频跳】
 *   按 mapping type（A/B）、duration、additional_position 查表，
 *   Type A 再叠加 l0（通常为 2 或 3）作为第一个 DMRS 位置。
 *   dual_symbol=1 时：每个块起始位置展开为 l 与 l+1。
 *
 * 【有频跳】
 *   分别填充 first_hop / second_hop，不可复用单跳表；
 *   每跳时长 duration/2，且至少 4 个符号。
 *
 * 【非法组合】
 *   例如双符号却 additional_position>1，返回 invalid_config。
 *
 * 【输出】
 *   first_hop  : 第一跳（或无频跳时全部）DMRS 符号集合
 *   second_hop : 第二跳 DMRS 符号集合（频跳时有效）
 */
ufeq_status_t ufeq_dmrs_get_symbol_positions(ufeq_pusch_mapping_type_t mapping_type, /* PUSCH 映射类型 A/B */
                                             Uchar duration,                          /* PUSCH 占用符号数 */
                                             Uchar l0,                                /* Type A 首个 DMRS 基准位置 */
                                             Uchar dmrs_additional_position,          /* 附加 DMRS 位置索引 0~3 */
                                             Uchar dual_symbol,                       /* 是否双符号 DMRS */
                                             Uchar frequency_hopping,                 /* 是否频跳 */
                                             ufeq_dmrs_symbol_set_t *first_hop,       /* 输出：第一跳符号集 */
                                             ufeq_dmrs_symbol_set_t *second_hop)      /* 输出：第二跳符号集 */
{
    /* 无频跳 Type A 查表：duration-4 行 × additional_position 列 × 最多 4 个附加位置 */
    static const Uchar g_single_a[11][4][4] = {
        {{0xff}, {0xff}, {0xff}, {0xff}}, {{0xff}, {0xff}, {0xff}, {0xff}},
        {{0xff}, {0xff}, {0xff}, {0xff}}, {{0xff}, {0xff}, {0xff}, {0xff}},
        {{0xff}, {7, 0xff}, {7, 0xff}, {7, 0xff}},
        {{0xff}, {7, 0xff}, {7, 0xff}, {7, 0xff}},
        {{0xff}, {9, 0xff}, {6, 9, 0xff}, {6, 9, 0xff}},
        {{0xff}, {9, 0xff}, {6, 9, 0xff}, {6, 9, 0xff}},
        {{0xff}, {9, 0xff}, {6, 9, 0xff}, {5, 8, 11, 0xff}},
        {{0xff}, {11, 0xff}, {7, 11, 0xff}, {5, 8, 11, 0xff}},
        {{0xff}, {11, 0xff}, {7, 11, 0xff}, {5, 8, 11, 0xff}}
    };
    /* 无频跳 Type B 查表：结构同 g_single_a */
    static const Uchar g_single_b[11][4][4] = {
        {{0xff}, {0xff}, {0xff}, {0xff}}, {{0xff}, {4, 0xff}, {4, 0xff}, {4, 0xff}},
        {{0xff}, {4, 0xff}, {4, 0xff}, {4, 0xff}}, {{0xff}, {4, 0xff}, {4, 0xff}, {4, 0xff}},
        {{0xff}, {6, 0xff}, {3, 6, 0xff}, {3, 6, 0xff}},
        {{0xff}, {6, 0xff}, {3, 6, 0xff}, {3, 6, 0xff}},
        {{0xff}, {8, 0xff}, {4, 8, 0xff}, {3, 6, 9, 0xff}},
        {{0xff}, {8, 0xff}, {4, 8, 0xff}, {3, 6, 9, 0xff}},
        {{0xff}, {10, 0xff}, {5, 10, 0xff}, {3, 6, 9, 0xff}},
        {{0xff}, {10, 0xff}, {5, 10, 0xff}, {3, 6, 9, 0xff}},
        {{0xff}, {10, 0xff}, {5, 10, 0xff}, {3, 6, 9, 0xff}}
    };
    const Uchar (*table)[4] = NULL; /* 指向当前 mapping type 的查表行 */
    Uchar i = 0;                   /* 遍历附加 DMRS 位置 */
    Uchar base = 0;                /* 频跳时第一跳基准符号 */
    Uchar start = 0;               /* 无频跳时首个 DMRS 符号 */
    Uchar hop_duration = 0;        /* 频跳时每跳符号数 duration/2 */
    Uchar base_count = 0;          /* 双符号展开前的符号个数 */

    if (first_hop == NULL || (frequency_hopping != 0 && second_hop == NULL)) {
        return ufeq_fail("dmrs", "invalid_arg");
    }
    memset(first_hop, 0, sizeof(*first_hop));
    if (second_hop != NULL) {
        memset(second_hop, 0, sizeof(*second_hop));
    }
    /* 参数范围合法性检查 */
    if ((mapping_type != ufeq_pusch_mapping_type_a && mapping_type != ufeq_pusch_mapping_type_b) ||
        duration < 1 || duration > 14 || dmrs_additional_position > 3 || dual_symbol > 1 ||
        frequency_hopping > 1) {
        return ufeq_fail("dmrs", "invalid_config");
    }
    /* 双符号 DMRS 仅支持 additional_position 0 或 1 */
    if (dual_symbol != 0 && dmrs_additional_position > 1) return ufeq_fail("dmrs", "invalid_config");

    if (frequency_hopping != 0) {
        /* ---- 频跳分支：分别填充 first_hop / second_hop ---- */
        hop_duration = (Uchar)(duration / 2);
        if (hop_duration < 4) return ufeq_fail("dmrs", "invalid_config");
        base = mapping_type == ufeq_pusch_mapping_type_a ? l0 : 0;
        if (mapping_type == ufeq_pusch_mapping_type_a && (l0 != 2 && l0 != 3)) return ufeq_fail("dmrs", "invalid_config");
        ufeq_push_symbol(first_hop, base);  /* 第一跳首个 DMRS */
        ufeq_push_symbol(second_hop, 0);    /* 第二跳首个 DMRS 固定为符号 0 */
        if (dmrs_additional_position == 1 && hop_duration >= 5) {
            /* 附加 DMRS：第一跳偏移 6（hop≥7）或 0；第二跳固定符号 4 */
            ufeq_push_symbol(first_hop, (Uchar)(base + (hop_duration >= 7 ? 6 : 0)));
            ufeq_push_symbol(second_hop, 4);
        }
        second_hop->hop_id = 1; /* 标记为第二跳 */
    } else {
        /* ---- 无频跳分支：查表获取附加 DMRS 位置 ---- */
        if (duration < 4 || (mapping_type == ufeq_pusch_mapping_type_a && (l0 != 2 && l0 != 3))) return ufeq_fail("dmrs", "invalid_config");
        table = mapping_type == ufeq_pusch_mapping_type_a ? g_single_a[duration - 4] : g_single_b[duration - 4];
        start = mapping_type == ufeq_pusch_mapping_type_a ? l0 : 0;
        ufeq_push_symbol(first_hop, start); /* 首个 DMRS 位置 */
        for (i = 0; i < 4 && table[dmrs_additional_position][i] != 0xff; ++i) {
            ufeq_push_symbol(first_hop, table[dmrs_additional_position][i]); /* 附加位置 */
        }
        if (dual_symbol != 0) {
            /* 双符号：每个已有位置 l 再插入 l+1 */
            base_count = first_hop->count;
            for (i = 0; i < base_count; ++i) ufeq_push_symbol(first_hop, (Uchar)(first_hop->symbols[i] + 1));
        }
    }
    first_hop->conditional_mask = (Ushort)(1 << dmrs_additional_position); /* 条件掩码供上层使用 */
    return ufeq_status_ok;
}
