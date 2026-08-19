#ifndef ufeq_h
#define ufeq_h

/**
 * UFEQ 公共 API（上行频域均衡库）
 *
 * 【职责】
 *   提供工作区管理、端到端处理入口，以及 Low-PAPR / DMRS 等协议辅助函数。
 *
 * 【内存约定】
 *   处理路径不在库内 malloc；调用方先按 ufeq_workspace_get_size 分配连续内存，
 *   再交给 ufeq_workspace_init 切分成各算法临时区。
 *
 * 【推荐调用顺序】
 *   config_init_default（或自填配置）
 *   -> workspace_get_size / 分配 memory
 *   -> workspace_init
 *   -> 可选 set_platform / set_log
 *   -> 填充 request / result 缓冲
 *   -> ufeq_process
 */

#include <stdbool.h>   /* bool 类型 */
#include <stdint.h>    /* 定宽整型 */

#include "ufeq_config.h"  /* 配置、请求、结果结构 */
#include "ufeq_status.h"  /* 统一返回状态枚举 */
#include "ufeq_tables.h"  /* DMRS 表结构与自检 */
#include "ufeq_types.h"   /* 基础类型、平台算子、统计 */

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- 工作区：按配置切分临时 buffer ---------- */

/**
 * 查询 workspace 所需字节数（含对齐）。
 *
 * @param config    算法配置，决定各临时区尺寸
 * @param size_byte 输出：所需连续内存字节数
 */
ufeq_status_t ufeq_workspace_get_size(const ufeq_config_t *config, Uint *size_byte);

/**
 * 用调用方提供的 memory 初始化 workspace，切分各算法临时区。
 *
 * @param config       算法配置
 * @param memory       调用方分配的连续内存首地址
 * @param memory_size  memory 字节数，须 >= get_size 返回值
 * @param workspace    输出：已切分缓冲的工作区对象
 */
ufeq_status_t ufeq_workspace_init(const ufeq_config_t *config,
                                  void *memory,
                                  Uint memory_size,
                                  ufeq_workspace_t *workspace);

/**
 * 注入平台算子（矩阵求逆/IDFT/atan/sin_cos/倒数），默认使用参考实现。
 *
 * @param workspace  已初始化工作区
 * @param ops        平台算子表；NULL 则保持当前/默认
 */
ufeq_status_t ufeq_workspace_set_platform(ufeq_workspace_t *workspace, const ufeq_platform_ops_t *ops);

/**
 * 可选日志回调；库内不直接 printf。同时作为库级告警出口（ufeq_warn）。
 *
 * @param workspace  已初始化工作区
 * @param callback   日志回调；NULL 则关闭日志
 * @param user       透传给 callback 的用户指针
 */
ufeq_status_t ufeq_workspace_set_log(ufeq_workspace_t *workspace,
                                     ufeq_log_callback_t callback,
                                     void *user);

/**
 * 读取保护/饱和等运行统计。
 *
 * @param workspace  已初始化工作区
 * @return           指向内部 stats 的只读指针，生命周期随 workspace
 */
const ufeq_stats_t *ufeq_workspace_get_stats(const ufeq_workspace_t *workspace);

/* ---------- 顶层处理 ---------- */

/**
 * 端到端处理入口
 *
 * 流水线顺序：
 *   AGC -> DMRS前校准 -> 信道插值 -> RUU/MRC-IRC -> MMSE
 *   -> 后频偏 -> IDFT -> 数据频偏 -> 层逆映射
 *   -> UCI demux1 -> 软解调 -> 解扰 -> UCI demux2
 *
 * 任一步失败则清空 result 中所有 count，result->status / 返回值为 error。
 * 细分失败原因通过日志告警（需 set_log），不占用多种返回状态。
 * equalized / SINR 在全部后级完成后再最终导出（避免中间态）。
 *
 * @param request    输入：配置 + 频域数据/信道估计等
 * @param result     输出：均衡符号、SINR、软比特等；调用方预分配缓冲
 * @param workspace  工作区（含各阶段临时缓冲）
 */
ufeq_status_t ufeq_process(const ufeq_request_t *request,
                           ufeq_result_t *result,
                           ufeq_workspace_t *workspace);

/**
 * 填充一份可用的默认配置（SISO/QPSK/CP-OFDM 等）。
 *
 * @param config  待初始化的配置结构
 */
ufeq_status_t ufeq_config_init_default(ufeq_config_t *config);

/* ---------- 协议辅助（可独立调用） ---------- */

/**
 * Low-PAPR 参考序列生成（3GPP TS 38.211 §5.2.2 / §5.2.3）
 *
 * type=1：Type1 基序列（phi 表直接构造）
 * type=2：Type2（短序列 phi 或二进制序列 + IDFT）
 * u: 组号 0..29；alpha: 循环移位相位；m_zc: 序列长度
 *
 * @param type          序列类型：1=Type1，2=Type2
 * @param u             组号 u，范围 0..29
 * @param v             组内基序列索引 v
 * @param alpha         循环移位相位（弧度）
 * @param delta         频域偏移参数
 * @param m_zc          ZC/序列长度（RE 数）
 * @param out           输出复数序列缓冲
 * @param out_capacity  out 可容纳的复数个数
 * @param out_count     输出：实际写入个数
 */
ufeq_status_t ufeq_low_papr_generate(Uchar type,
                                     Uchar u,
                                     Uchar v,
                                     float alpha,
                                     Uchar delta,
                                     Ushort m_zc,
                                     ufeq_cfloat_t *out,
                                     Ushort out_capacity,
                                     Ushort *out_count);

/**
 * 定点路径用的 16QAM 调制距离常量（由 Q(N,1) 常数移位得到）。
 *
 * @param n_mmse_shift  MMSE 输出定标右移位数
 */
float ufeq_demod_distance_16qam(Uchar n_mmse_shift);

/** 定点路径用的 64QAM 调制距离常量。 */
float ufeq_demod_distance_64qam(Uchar n_mmse_shift);

/** 定点路径用的 256QAM 调制距离常量。 */
float ufeq_demod_distance_256qam(Uchar n_mmse_shift);

/**
 * PUSCH DMRS 端口频域参数（CDM 组、delta、OCC）。
 *
 * @param config_type  DMRS 配置类型：type1 / type2
 * @param port         天线端口号
 * @param dual_symbol  是否双符号 DMRS（1=是）
 * @param param        输出：CDM 组、delta、频/时 OCC 权重
 */
ufeq_status_t ufeq_dmrs_get_port_param(ufeq_dmrs_config_type_t config_type,
                                       Uchar port,
                                       Uchar dual_symbol,
                                       ufeq_dmrs_port_param_t *param);

/**
 * PUSCH DMRS 时域符号位置查表
 *
 * frequency_hopping!=0 时分别填充 first_hop / second_hop，不可复用单跳结果。
 * dual_symbol=1 时每个位置展开为连续两个符号。
 *
 * @param mapping_type              PUSCH 映射类型 A/B
 * @param duration                  分配符号数（时域长度）
 * @param l0                        首个 DMRS 符号基准位置
 * @param dmrs_additional_position  附加 DMRS 位置索引
 * @param dual_symbol               是否双符号 DMRS
 * @param frequency_hopping         是否频域跳频（0=单跳）
 * @param first_hop                 输出：第一跳 DMRS 符号集合
 * @param second_hop                输出：第二跳 DMRS 符号集合（跳频时有效）
 */
ufeq_status_t ufeq_dmrs_get_symbol_positions(ufeq_pusch_mapping_type_t mapping_type,
                                             Uchar duration,
                                             Uchar l0,
                                             Uchar dmrs_additional_position,
                                             Uchar dual_symbol,
                                             Uchar frequency_hopping,
                                             ufeq_dmrs_symbol_set_t *first_hop,
                                             ufeq_dmrs_symbol_set_t *second_hop);

#ifdef __cplusplus
}
#endif

#endif /* ufeq_h */
