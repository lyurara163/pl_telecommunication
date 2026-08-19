#ifndef ufeq_types_h
#define ufeq_types_h

/**
 * UFEQ 基础类型与枚举
 *
 * Stage-1 参考实现以浮点为主；cint16/cint32 用于与定点接口对接。
 * 调制枚举值为每符号比特数（QPSK=2 … 256QAM=8），便于 soft_bit 计数。
 *
 * 无符号整型命名对齐外部平台：Uchar / Ushort / Uint。
 * PC 参考构建通过下方 typedef 映射到 stdint；DSP 平台可定义
 * UFEQ_USE_PLATFORM_TYPES，并提供含 Uchar/Ushort/Uint 的平台头。
 */

#include <stdbool.h>   /* bool 类型 */
#include <stdint.h>    /* 定宽整型 */

#include "ufeq_status.h"  /* ufeq_status_t */

#ifdef UFEQ_USE_PLATFORM_TYPES
#include "ufeq_platform_types.h"  /* DSP 平台 Uchar/Ushort/Uint 等 */
#else
typedef uint8_t Uchar;   /* 无符号 8 位，对齐平台命名 */
typedef uint16_t Ushort; /* 无符号 16 位 */
typedef uint32_t Uint;   /* 无符号 32 位 */
typedef int8_t Schar;    /* 有符号 8 位（如 AGC 因子） */
typedef int32_t Sint;    /* 有符号 32 位（平台 padding） */
#endif

/** 复数浮点（算法工作区主类型，内部 MMSE/插值等均用此类型）。 */
typedef struct {
    float re; /* 实部 */
    float im; /* 虚部 */
} ufeq_cfloat_t;

/** 复数定点 32-bit（均衡输出、信道估计等对外接口）。 */
typedef struct {
    int32_t re; /* 实部，带 Q 格式定标 */
    int32_t im; /* 虚部 */
} ufeq_cint32_t;

/** 复数定点 16-bit（接收 IQ 输入，与 freq_data 解包一致）。 */
typedef struct {
    int16_t re;
    int16_t im;
} ufeq_cint16_t;

/** 行主序复数矩阵视图（不拥有 data 内存）。 */
typedef struct {
    Ushort row_num;       /* 行数 */
    Ushort col_num;       /* 列数 */
    Ushort stride;        /* 行 stride（>= col_num，允许子矩阵） */
    ufeq_cfloat_t *data;  /* 指向 row_num*stride 元素的缓冲 */
} ufeq_cmatrix_f32_t;

/** 无重复索引列表（如 DC RE、有效天线列表）。 */
typedef struct {
    Uint count;    /* 有效索引个数 */
    Ushort *value; /* 索引值数组，长度 >= count */
} ufeq_index_list_t;

/** NR 上行波形类型。 */
typedef enum {
    ufeq_waveform_cp_ofdm = 0,    /* 常规 CP-OFDM */
    ufeq_waveform_dft_s_ofdm = 1  /* DFT-s-OFDM（变换预编码） */
} ufeq_waveform_t;

/**
 * 均衡器模式。
 * MRC=对角近似；IRC=完整 Ruu^{-1}；AUTO 按相关度门限切换。
 */
typedef enum {
    ufeq_equalizer_mrc = 0,  /* 最大比合并（忽略干扰相关） */
    ufeq_equalizer_irc = 1,  /* 干扰抑制合并（使用 Ruu^{-1}） */
    ufeq_equalizer_auto = 2  /* 根据天线相关度自动选择 MRC/IRC */
} ufeq_equalizer_mode_t;

/** 调制方式；枚举数值 = Qm（每符号比特数）。 */
typedef enum {
    ufeq_mod_pi_2_bpsk = 1, /* π/2-BPSK，1 bit/符号 */
    ufeq_mod_qpsk = 2,      /* QPSK，2 bit/符号 */
    ufeq_mod_16qam = 4,     /* 16QAM，4 bit/symbol */
    ufeq_mod_64qam = 6,     /* 64QAM，6 bit/symbol */
    ufeq_mod_256qam = 8     /* 256QAM，8 bit/symbol */
} ufeq_modulation_t;

/** PUSCH DMRS 配置类型（Type1 / Type2 频域模式）。 */
typedef enum {
    ufeq_dmrs_config_type1 = 1,
    ufeq_dmrs_config_type2 = 2
} ufeq_dmrs_config_type_t;

/** PUSCH 时域映射类型（Type A / Type B 起始符号规则不同）。 */
typedef enum {
    ufeq_pusch_mapping_type_a = 0,
    ufeq_pusch_mapping_type_b = 1
} ufeq_pusch_mapping_type_t;

/**
 * 运行时保护/饱和计数，便于联调与回归。
 * 通过 ufeq_workspace_get_stats 只读访问。
 */
typedef struct {
    Uint matrix_guard_count;        /* 矩阵运算中对角/分母保护触发次数 */
    Uint matrix_inverse_fail_count; /* Hermitian 求逆失败次数 */
    Uint sinr_guard_count;          /* SINR 钳位到 [rho_min, rho_max] 次数 */
    Uint saturation_count;          /* 定点饱和（saturate）次数 */
    Uint no_cfo_sample_count;       /* 数据频偏无有效环形样本次数 */
    Uint invalid_dc_index_count;    /* DC 索引越界丢弃次数 */
    Uint invalid_table_index_count; /* 查表索引非法次数 */
    Uint mrc_select_count;          /* AUTO 模式选中 MRC 次数 */
    Uint irc_select_count;          /* AUTO 模式选中 IRC 次数 */
    Uint agc_max_shift;             /* AGC 对齐观测到的最大移位 */
} ufeq_stats_t;

/** 日志级别：细分失败原因走 warn，不占用返回状态码。 */
enum {
    ufeq_log_level_info = 0,  /* 一般信息 */
    ufeq_log_level_warn = 1,  /* 可恢复异常/降级（对应 ufeq_warn） */
    ufeq_log_level_error = 2  /* 致命/参数错误 */
};

/**
 * 日志回调函数类型。
 *
 * @param level   ufeq_log_level_*
 * @param module  模块名，如 "channel"、"mmse"
 * @param message 英文短码或描述，如 "invalid_arg"
 * @param user    set_log 时传入的用户指针
 */
typedef void (*ufeq_log_callback_t)(int level, const char *module, const char *message, void *user);

/**
 * 平台可替换算子表。
 * 默认由 ufeq_platform_ref_ops() 提供；DSP 集成时可注入优化实现。
 */
typedef struct {
    /**
     * Hermitian 正定矩阵求逆（用于 Ruu^{-1}、r_hh^{-1}）。
     *
     * @param in         输入 Hermitian 矩阵（行主序，dim×dim）
     * @param out        输出逆矩阵
     * @param dim        矩阵维数
     * @param kernel_dim 有效核维数（<= dim，用于部分天线失效）
     * @param diag_load  对角加载量，改善数值稳定性
     */
    ufeq_status_t (*matrix_inverse_hermitian)(const ufeq_cfloat_t *in,
                                              ufeq_cfloat_t *out,
                                              Ushort dim,
                                              Ushort kernel_dim,
                                              float diag_load);
    /**
     * 逆离散傅里叶变换（DFT-s-OFDM 解预编码）。
     * 长度须为 2^a2 * 3^a3 * 5^a5 形式。
     */
    ufeq_status_t (*idft)(const ufeq_cfloat_t *in, ufeq_cfloat_t *out, Ushort length);
    /** 四象限反正切，用于相位/频偏估计 */
    float (*atan2_f32)(float y, float x);
    /** 同步计算 sin/cos，避免多次 libm 调用 */
    void (*sin_cos_f32)(float phase, float *sin_v, float *cos_v);
    /** 带保护的倒数，分母过小时钳位到 ufeq_diag_protect */
    float (*reciprocal_f32)(float x);
} ufeq_platform_ops_t;

#endif /* ufeq_types_h */
