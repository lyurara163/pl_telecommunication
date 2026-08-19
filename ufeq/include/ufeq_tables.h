#ifndef ufeq_tables_h
#define ufeq_tables_h

/**
 * DMRS 端口/符号位置相关表结构，以及协议表自检入口。
 *
 * 表数据由 ufeq_tables.c 实现；本头仅暴露查表 API 所需类型。
 */

#include "ufeq_config.h"  /* ufeq_max_symb_slot 等常量 */
#include "ufeq_types.h"   /* Uchar、ufeq_status_t 等 */

/**
 * 单天线端口的 CDM 组、频域偏移与 OCC 权重。
 * 对应 3GPP TS 38.211 PUSCH DMRS 端口参数。
 */
typedef struct {
    Uchar cdm_group; /* CDM 组号（正交覆盖分组） */
    Uchar delta;     /* 频域 comb 偏移 delta */
    int8_t w_f[2];   /* 频域 OCC 权重，双符号时 [0]/[1] 对应两符号 */
    int8_t w_t[2];   /* 时域 OCC 权重 */
} ufeq_dmrs_port_param_t;

/**
 * DMRS 时域符号集合（含跳频/条件位掩码）。
 * 由 ufeq_dmrs_get_symbol_positions 填充。
 */
typedef struct {
    Uchar count;                          /* 有效 DMRS 符号个数 */
    Uchar symbols[ufeq_max_symb_slot];  /* 符号索引列表，长度 count */
    Ushort conditional_mask;              /* 条件附加 DMRS 位掩码（协议查表用） */
    Uchar hop_id;                         /* 跳频标识：0=第一跳，1=第二跳 */
} ufeq_dmrs_symbol_set_t;

/**
 * 协议常量表自检（端口表、符号位置表等）。
 * 可在启动或单元测试中调用，失败返回 ufeq_status_error。
 */
ufeq_status_t ufeq_tables_self_check(void);

#endif /* ufeq_tables_h */
