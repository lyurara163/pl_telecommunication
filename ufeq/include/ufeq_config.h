#ifndef ufeq_config_h
#define ufeq_config_h

/**
 * UFEQ 配置、请求与结果结构
 *
 * 索引约定（与算法文档一致）：
 *   rx_index      = ((((symbol*n_rb)+rb)*n_sc_rb+re)*n_rx+rx)
 *   channel_index = (((((symbol*n_rb)+rb)*n_sc_rb+re)*n_rx+rx)*n_layer+layer)
 *   layer_index   = ((((symbol*n_rb)+rb)*n_sc_rb+re)*n_layer+layer)
 */

#include "ufeq_types.h"  /* 基础类型与枚举 */

/* ---------- 实现上限与数值保护常量 ---------- */
#define ufeq_max_rx_ant      16   /* 最大接收天线数（算法上限） */
#define ufeq_max_layer       16   /* 最大空间层数（5G 常用至 8） */
#define ufeq_max_symb_slot   14   /* 每时隙 OFDM 符号数上限（NR 正常 CP） */
#define ufeq_max_sc_rb       12   /* 每 RB 子载波数（NR 固定 12） */
#define ufeq_sch_freq_max_rb    264 /* 平台频域 unit 最大 RB（100MHz 量级） */
#define ufeq_sch_freq_max_slot  16  /* 平台 slot 维最大个数 */
#define ufeq_sch_freq_max_ant   64  /* 平台天线维（索引步长用） */
#define ufeq_sch_freq_max_symb  14  /* 平台符号维 = ant 步长 */
#define ufeq_max_dmrs_symb   6    /* 单用户 DMRS 符号数上限 */
#define ufeq_max_codeword    2    /* 最大码字数（8 层 MIMO 时为 2） */
#define ufeq_diag_protect    1e-9 /* 对角/分母非正保护，避免除零 */
#define ufeq_rhh_diag_load   1e-5 /* r_hh 对角线额外加载，改善条件数 */
#define ufeq_rho_min         1.0  /* SINR 下限（约 -15 dB 量级保护） */
#define ufeq_rho_max         32767.0 /* SINR 上限，与定点 rho 量纲对齐 */

/**
 * 算法配置：与每次请求的数据指针分离。
 * bypass_* = true 时跳过对应运算，但输出布局与定标语义不变。
 */
typedef struct {
    Ushort n_rx;              /* 接收天线数 */
    Ushort n_layer;           /* 空间层数（发送层） */
    Ushort n_rb;              /* 调度 RB 数 */
    Ushort n_sc_rb;           /* 每 RB 子载波数，通常 12 */
    Ushort n_symb_slot;       /* 时隙内 OFDM 符号数，通常 14 */
    Ushort m_sc_pusch;        /* 每符号有效 RE 数 = n_rb * n_sc_rb */
    Uchar n_mmse_shift;       /* 均衡输出定标，推荐 5，范围 3~6 */
    Uchar n_sinr_shift;       /* rho 定标，推荐 11，范围 10~12 */
    Uchar n_sb_shift;         /* 软比特输出右移，0~7 */
    Uchar codeword_count;     /* 码字数：1~4 层为 1，5~8 层为 2；0 表示按层数推断 */
    Ushort n_rnti;            /* 解扰 c_init = n_rnti*2^15 + n_id */
    Ushort n_id;              /* 小区 ID，参与解扰与 DMRS 序列 */
    Ushort n_rs;              /* DMRS 符号个数 */
    Ushort dmrs_symbol[ufeq_max_dmrs_symb]; /* DMRS 所在 OFDM 符号索引（升序） */
    Ushort dmrs_ref_symbol;   /* 前校准相位参考符号（通常取首个 DMRS） */
    ufeq_waveform_t waveform; /* 波形：CP-OFDM 或 DFT-s-OFDM */
    ufeq_modulation_t modulation[ufeq_max_codeword]; /* 每码字调制阶数 */
    ufeq_equalizer_mode_t equalizer_mode;            /* MRC / IRC / AUTO */
    float corr_threshold;       /* AUTO 模式下 IRC/MRC 切换相关度门限 */
    float corr_hysteresis;      /* 迟滞带宽，避免模式在门限附近抖动 */
    float ruu_add_coeff;        /* RUU 对角加载系数（ruu_add_switch=1 时生效） */
    float ruu_scale;            /* 非对角元素缩放系数 */
    float noise_power_threshold; /* 噪声功率门限，用于 RUU 估计判据 */
    float th_in;                /* 数据频偏环形样本内门限（|phase| 判定） */
    float th_out;               /* 数据频偏环形样本外门限（迟滞退出） */
    float sinr_threshold_alpha; /* rho 门限系数，默认 256 表示 1.0（Q8） */
    float th_mcs;               /* MCS 对应 SNR 门限，用于软解调保护 */
    Uchar ruu_add_switch;     /* 1=启用对角加载/非对角缩放 */
    Uchar reuse_odd_sinr;     /* 1=奇数符号 SINR 复用相邻偶数符号 */
    Uchar pusch_uci_qpsk_sinr1_flag; /* UCI QPSK 特殊 SINR 处理标志 */
    Uchar pusch_harq_a_len;   /* ACK 信息比特数；<=2 走特殊替换路径 */
    Uchar pusch_csi1_a_len;   /* CSI Part1 信息比特数 */
    Uchar pusch_csi2_a_len;   /* CSI Part2 信息比特数 */
    Uchar csi2_rank;          /* CSI2 秩指示相关配置 */
    Uchar fixed_ri;           /* 固定 RI 值（测试/旁路用） */
    bool bypass_agc;              /* 1=跳过 AGC 对齐 */
    bool bypass_dmrs_freq_offset; /* 1=跳过 DMRS 频偏前校准 */
    bool bypass_dmrs_time_filter; /* 1=跳过 DMRS 时域插值（旁路复制） */
    bool bypass_ruu;              /* 1=跳过 RUU 估计/准备 */
    bool bypass_ruu_adjust;       /* 1=跳过 RUU 对角加载与非对角缩放 */
    bool bypass_inv_ruu;          /* 1=跳过 Ruu 求逆（使用外部 ruu_inv） */
    bool bypass_freq_offset_2;    /* 1=跳过均衡后频偏校正 */
    bool bypass_idft;             /* 1=跳过 DFT-s-OFDM 的 IDFT 解预编码 */
    bool bypass_data_freq_offset; /* 1=跳过数据符号残余频偏处理 */
    bool bypass_layer_demap;      /* 1=跳过层逆映射 */
    bool bypass_demux_1;          /* 1=跳过 UCI 第一次解复用 */
    bool bypass_demod;            /* 1=跳过软解调 */
    bool bypass_descramble;       /* 1=跳过解扰 */
    bool bypass_demux_2;          /* 1=跳过 UCI 第二次解复用 */
    bool bypass_symbol_process;   /* 1=跳过均衡主链，直接吃已均衡数据 */
} ufeq_config_t;

/**
 * 平台频域数据 unit（与 gde_sch_freq_data[][][] 元素一致，Size 16K）
 *
 * freq_data：Uint 打包 IQ（低 16bit=I，高 16bit=Q），按 rb*12+re 排布
 * agc：该 (slot, ant, symb) 单元的 AGC 因子
 */
typedef struct {
    Uint freq_data[ufeq_sch_freq_max_rb * 12]; /* 264rb * 12re，频域 IQ 打包数组 */
    Schar agc;              /* 该单元 AGC 右移/缩放因子（有符号） */
    Schar pad0[3];            /* 字节对齐填充 */
    Sint pad1[927];           /* 补齐至平台固定 16K 结构体大小 */
} ufeq_sch_freq_data_unit_t;

/** 处理参数（与大数据缓冲分离，便于复用配置） */
typedef struct {
    const ufeq_config_t *config; /* 指向本次处理使用的算法配置 */
    Ushort start_rb; /* sch_freq_data.freq_data 内调度 RB 起始下标（相对 unit 内 0） */
} ufeq_param_t;

/** 处理输入数据缓冲（每次请求可不同，配置通常复用） */
typedef struct {
    /**
     * 指向 gde_sch_freq_data[slot][0][0]
     * 布局 [ant][symb]，访问 index = ant * ufeq_sch_freq_max_symb + symb
     */
    const ufeq_sch_freq_data_unit_t *sch_freq_data;
    const ufeq_cint32_t *channel_est; /* DMRS 信道估计，定标约 Q(N,5) */
    Uint channel_est_count;         /* channel_est 复数元素个数 */
    const ufeq_cfloat_t *ruu;         /* 可选：外部干扰协方差矩阵（按 RE 或全局） */
    const ufeq_cfloat_t *ruu_inv;     /* 可选：外部已求逆 Ruu，bypass_inv_ruu 时使用 */
    const int16_t *dmrs_phase;        /* 可选外部相位（当前主路径自行估计） */
    const int16_t *post_cfo_phase;    /* 均衡后按符号的相位补偿表，定标 Q(N,3) */
    const int8_t *pusch_re_flag;      /* RE 类型：-2/-1/0/1/2/3（UCI/数据/占位等） */
    Uint pusch_re_flag_count;       /* pusch_re_flag 元素个数 */
    const Ushort *dc_index;           /* DC 干扰 RE 线性索引，均衡后置零 x_hat/rho */
    Uint dc_count;                    /* dc_index 个数 */
    const Uchar *rx_ant_mat_flag;     /* 有效天线标志，0=该天线无效/不参与合并 */
    const ufeq_cfloat_t *residual_sample; /* 用于估计 RUU 的残差样本（可选） */
    Uint residual_sample_count;       /* 残差样本个数 */
    const Uchar *csi2_rank_vec;       /* CSI2 秩向量（按 RE 或码字） */
    Uint csi2_rank_vec_count;         /* csi2_rank_vec 长度 */
    /* UCI≤2bit 时码字内占位比特索引（3GPP 38.211 φx/φy）；可为 NULL */
    const Uint *phi_x_index;          /* φx 占位 RE/比特索引表 */
    Uint phi_x_count;               /* phi_x_index 长度 */
    const Uint *phi_y_index;          /* φy 占位 RE/比特索引表 */
    Uint phi_y_count;               /* phi_y_index 长度 */
    /* bypass_symbol_process=1 时的预均衡输入 */
    const ufeq_cint32_t *pre_equalized; /* 外部已均衡符号，跳过 MMSE 主链 */
    Uint pre_equalized_count;         /* pre_equalized 复数个数 */
} ufeq_data_t;

/** 单次处理输入：参数与数据分开，便于同一 config 多次调用。 */
typedef struct {
    ufeq_param_t param; /* 配置与 RB 起始等轻量参数 */
    ufeq_data_t data;   /* 频域 IQ、信道、UCI 标志等大数据指针 */
} ufeq_request_t;

/** 单次处理输出：失败时所有 count 置 0，status 为 error。 */
typedef struct {
    ufeq_cint32_t *equalized_data;    /* 均衡后符号（定点输出） */
    Uint equalized_capacity;        /* equalized_data 容量（复数个数） */
    Uint equalized_count;           /* 实际写入均衡符号数 */
    int32_t *sinr;                    /* 每 RE/层 SINR（定点 rho） */
    Uint sinr_capacity;             /* sinr 缓冲容量 */
    Uint sinr_count;                /* 实际 SINR 个数 */
    int8_t *sch_soft_bit;               /* 调度数据软比特 LLR */
    Uint sch_capacity;                /* sch 软比特缓冲容量 */
    Uint sch_count;                   /* 实际 sch 软比特数 */
    int8_t *ack_soft_bit;               /* HARQ-ACK 软比特 */
    Uint ack_capacity;
    Uint ack_count;
    int8_t *csi1_soft_bit;              /* CSI Part1 软比特 */
    Uint csi1_capacity;
    Uint csi1_count;
    int8_t *csi2_soft_bit;              /* CSI Part2 软比特 */
    Uint csi2_capacity;
    Uint csi2_count;
    Uchar *scram_ack;                   /* ACK 解扰序列（调试/下游用） */
    Uchar *scram_csi1;                  /* CSI1 解扰序列 */
    Uchar *scram_csi2;                  /* CSI2 解扰序列 */
    Uint *index_ack;                    /* ACK 比特到 RE/符号的映射索引 */
    Uint *index_csi1;                   /* CSI1 映射索引 */
    Uint *index_csi2;                   /* CSI2 映射索引 */
    ufeq_cfloat_t *uci_syms;            /* 第一次解复用后的 UCI 符号（浮点中间态） */
    Uint uci_syms_capacity;
    Uint uci_syms_count;
    float *uci_sinr;                    /* UCI RE 对应 SINR（浮点） */
    Uint uci_sinr_capacity;
    Uint uci_sinr_count;
    float data_cfo_phase;               /* 数据符号残余相位估计（弧度） */
    ufeq_equalizer_mode_t selected_mode; /* AUTO 模式下实际选用的均衡模式 */
    float max_corr;                       /* 天线间最大相关度（AUTO 判据） */
    float avg_corr;                       /* 天线间平均相关度 */
    ufeq_status_t status;                 /* 与函数返回值一致的细粒度状态副本 */
} ufeq_result_t;

/** 工作区不透明类型，完整定义见 ufeq_internal.h（库内私有）。 */
typedef struct ufeq_workspace_s ufeq_workspace_t;

#endif /* ufeq_config_h */
