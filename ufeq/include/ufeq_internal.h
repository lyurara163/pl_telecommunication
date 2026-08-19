#ifndef ufeq_internal_h
#define ufeq_internal_h

/**
 * 库内私有头：工作区布局、索引公式、复数/定点工具、模块入口声明。
 * 外部应用应只包含 ufeq.h，勿直接依赖本头。
 */

#include "ufeq_config.h"  /* 配置与请求/结果 */
#include "ufeq_types.h"   /* 复数类型与平台算子 */

/**
 * 告警：细节原因走日志；未 set_log 时为空操作。
 * 返回值仅用 ok/error，message 为英文短码便于 grep。
 */
void ufeq_warn(const char *module, const char *message);

/** 等价于 ufeq_warn + 返回 ufeq_status_error，模块内统一失败出口。 */
ufeq_status_t ufeq_fail(const char *module, const char *message);

/**
 * 工作区完整布局：各阶段临时缓冲从 memory 一次性切分，处理路径不 malloc。
 */
struct ufeq_workspace_s {
    const ufeq_config_t *config;       /* 指向 init 时绑定的配置（只读） */
    const ufeq_platform_ops_t *ops;    /* 平台算子表，默认 ref */
    ufeq_stats_t stats;                /* 运行统计累加器 */
    ufeq_log_callback_t log_callback;  /* 可选日志回调 */
    void *log_user;                    /* 日志用户指针 */

    Uchar *memory;      /* workspace_init 切分起点，整块由调用方分配 */
    Uint memory_size; /* memory 字节数 */

    /* ---------- 主链临时：RX / 信道 / 均衡符号与 SINR ---------- */
    ufeq_cfloat_t *rx_work;        /* AGC 对齐后的接收频域数据（浮点） */
    ufeq_cfloat_t *channel_work;   /* DMRS 前校准后的信道估计 */
    ufeq_cfloat_t *channel_interp; /* 时域插值后全符号信道 H(l) */
    ufeq_cfloat_t *equalized_work; /* MMSE 均衡输出（浮点中间） */
    float *sinr_work;              /* 每 RE/层 SINR 工作缓冲（浮点 rho） */
    float *f_dft; /* DFT-s：每层每 RE 的噪声/归一化因子 f_dft */
    float *f_t;   /* DFT-s：每 OFDM 符号的时域归一化 f_t */
    float *f_cp;  /* CP-OFDM：每 RE 输出缩放 (1+rho)/rho */

    /* ---------- MMSE 单 RE 小矩阵缓冲（按最大天线/层分配） ---------- */
    ufeq_cfloat_t *ruu_work;      /* 当前 RE 的干扰协方差 Ruu */
    ufeq_cfloat_t *ruu_inv_work;  /* Ruu 求逆结果 */
    ufeq_cfloat_t *h_bar_h;       /* 有效信道相关 H^H H 或类似中间量 */
    ufeq_cfloat_t *r_hh;          /* 接收相关矩阵 r_hh */
    ufeq_cfloat_t *r_hh_inv;      /* r_hh 逆 */
    ufeq_cfloat_t *x_tilde;       /* 匹配滤波输出 \tilde{x} */
    ufeq_cfloat_t *x_hat;         /* MMSE 估计符号 */
    ufeq_cfloat_t *idft_tmp;      /* IDFT 输入/输出复用缓冲 */
    ufeq_cfloat_t *codeword_symbol[ufeq_max_codeword]; /* 按码字聚合的符号 */
    float *codeword_sinr[ufeq_max_codeword];           /* 按码字聚合的 SINR */
    Uint codeword_symbol_count[ufeq_max_codeword];   /* 各码字符号计数 */
    Uint soft_bit_offset[ufeq_max_codeword]; /* 双码字解扰/软比特分段起点 */
    Uint soft_bit_count[ufeq_max_codeword];  /* 各码字软比特数 */
    Uint valid_re_flag_capacity;             /* valid_re_flag 分配长度 */
    Uchar effective_codeword_count; /* 运行时推断码字数，不改写 const config */

    int8_t *soft_bit_tmp;  /* 软解调 LLR 临时缓冲 */
    Uchar *scram_tmp;      /* 解扰比特临时 */
    Uint *index_buffer;    /* UCI 索引映射临时 */
    int8_t *valid_re_flag; /* 有效数据 RE 标志（跳过 UCI/占位） */

    Uint rx_work_count;           /* rx_work 元素个数 */
    Uint channel_work_count;      /* channel_work 元素个数 */
    Uint equalized_count;         /* equalized_work 元素个数 */
    Uint soft_bit_tmp_capacity;   /* soft_bit_tmp 容量 */

    ufeq_equalizer_mode_t selected_mode; /* AUTO 模式下本次选用的 MRC/IRC */
    float dmrs_delta_theta;              /* DMRS 前校准估计的频偏相位斜率 */
    float data_cfo_phase;                /* 数据符号残余 CFO 相位 */
    bool rx_work_owns;        /* 1=rx_work 由 workspace 拥有（否则指向外部） */
    bool channel_work_owns;   /* 1=channel_work 由 workspace 拥有 */
};


/* ---------- 平台频域 unit 访问 ---------- */

/**
 * 按 [天线][符号] 二维索引取 sch_freq_data 单元指针。
 *
 * @param base  指向 gde_sch_freq_data[slot][0][0] 的基址
 * @param ant   天线索引
 * @param symb  OFDM 符号索引
 */
static inline const ufeq_sch_freq_data_unit_t *ufeq_sch_freq_unit(
    const ufeq_sch_freq_data_unit_t *base, Ushort ant, Ushort symb)
{
    return &base[(Uint)ant * ufeq_sch_freq_max_symb + symb]; /* 行主序：ant 块内 symb */
}

/** Uint 打包 IQ -> cint32（低 16=I，高 16=Q，符号扩展至 32 位）。 */
static inline ufeq_cint32_t ufeq_unpack_freq_uint(Uint word)
{
    ufeq_cint32_t s = {0};
    s.re = (int16_t)(word & 0xFFFFu);              /* 低 16 位 I */
    s.im = (int16_t)((word >> 16) & 0xFFFFu);      /* 高 16 位 Q */
    return s;
}

/** cint16 分量打包为平台 Uint 频域字（低 I 高 Q）。 */
static inline Uint ufeq_pack_freq_iq(int16_t re, int16_t im)
{
    return ((Uint)(uint16_t)re) | (((Uint)(uint16_t)im) << 16);
}

/* ---------- 缓冲线性索引（与 ufeq_config.h 约定一致） ---------- */

/**
 * 接收数据线性下标：symbol, rb, re, rx。
 * index = ((((symbol*n_rb)+rb)*n_sc_rb+re)*n_rx+rx)
 */
static inline Uint ufeq_rx_index(Ushort symbol,
                                   Ushort rb,
                                   Ushort re,
                                   Ushort rx,
                                   const ufeq_config_t *cfg)
{
    return (((((Uint)symbol * cfg->n_rb) + rb) * cfg->n_sc_rb + re) * cfg->n_rx) + rx;
}

/**
 * 信道矩阵线性下标：含 layer 维。
 * index = (((((symbol*n_rb)+rb)*n_sc_rb+re)*n_rx+rx)*n_layer+layer)
 */
static inline Uint ufeq_channel_index(Ushort symbol,
                                        Ushort rb,
                                        Ushort re,
                                        Ushort rx,
                                        Ushort layer,
                                        const ufeq_config_t *cfg)
{
    return ((((((Uint)symbol * cfg->n_rb) + rb) * cfg->n_sc_rb + re) * cfg->n_rx + rx)
            * cfg->n_layer)
           + layer;
}

/**
 * 层映射数据线性下标（无 rx 维，均衡后 per-layer RE）。
 * index = ((((symbol*n_rb)+rb)*n_sc_rb+re)*n_layer+layer)
 */
static inline Uint ufeq_layer_index(Ushort symbol,
                                      Ushort rb,
                                      Ushort re,
                                      Ushort layer,
                                      const ufeq_config_t *cfg)
{
    return (((((Uint)symbol * cfg->n_rb) + rb) * cfg->n_sc_rb + re) * cfg->n_layer) + layer;
}

/**
 * 按 PUSCH 线性子载波 sc（0..m_sc_pusch-1）的层索引。
 * index = ((symbol*m_sc_pusch)+sc)*n_layer+layer
 */
static inline Uint ufeq_sc_index(Ushort symbol, Ushort sc, Ushort layer, const ufeq_config_t *cfg)
{
    return (((Uint)symbol * cfg->m_sc_pusch) + sc) * cfg->n_layer + layer;
}

/** 构造复数常量 (re, im)。 */
static inline ufeq_cfloat_t ufeq_cf(float re, float im)
{
    ufeq_cfloat_t v = {0.0, 0.0};
    v.re = re;
    v.im = im;
    return v;
}

/** 复数加法 a + b。 */
static inline ufeq_cfloat_t ufeq_cf_add(ufeq_cfloat_t a, ufeq_cfloat_t b)
{
    return ufeq_cf(a.re + b.re, a.im + b.im);
}

/** 复数减法 a - b。 */
static inline ufeq_cfloat_t ufeq_cf_sub(ufeq_cfloat_t a, ufeq_cfloat_t b)
{
    return ufeq_cf(a.re - b.re, a.im - b.im);
}

/** 复数乘法 a * b。 */
static inline ufeq_cfloat_t ufeq_cf_mul(ufeq_cfloat_t a, ufeq_cfloat_t b)
{
    return ufeq_cf(a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re);
}

/** 复数标量乘法 s * a。 */
static inline ufeq_cfloat_t ufeq_cf_scale(ufeq_cfloat_t a, float s)
{
    return ufeq_cf(a.re * s, a.im * s);
}

/** 复数共轭 conj(a)。 */
static inline ufeq_cfloat_t ufeq_cf_conj(ufeq_cfloat_t a)
{
    return ufeq_cf(a.re, -a.im);
}

/** 复数模平方 |a|^2 = re^2 + im^2。 */
static inline float ufeq_cf_abs2(ufeq_cfloat_t a)
{
    return a.re * a.re + a.im * a.im;
}

/** 定点 cint32 转浮点，乘以 scale（通常为 2^{-n}）。 */
static inline ufeq_cfloat_t ufeq_cint32_to_cf(ufeq_cint32_t v, float scale)
{
    return ufeq_cf((float)v.re * scale, (float)v.im * scale);
}

/**
 * 浮点转定点 cint32，带四舍五入与 int32 饱和。
 *
 * @param v      输入复数
 * @param scale  乘性定标（输出 ≈ round(v * scale)）
 */
static inline ufeq_cint32_t ufeq_cf_to_cint32(ufeq_cfloat_t v, float scale)
{
    ufeq_cint32_t o = {0};
    float re = 0.0;
    float im = 0.0;
    re = v.re * scale;
    im = v.im * scale;
    if (re > 2147483647.0) { /* 实部上溢钳位 */
        re = 2147483647.0;
    }
    if (re < -2147483648.0) { /* 实部下溢钳位 */
        re = -2147483648.0;
    }
    if (im > 2147483647.0) { /* 虚部上溢钳位 */
        im = 2147483647.0;
    }
    if (im < -2147483648.0) { /* 虚部下溢钳位 */
        im = -2147483648.0;
    }
    o.re = (int32_t)(re >= 0.0 ? re + 0.5 : re - 0.5); /* 四舍五入到 int32 */
    o.im = (int32_t)(im >= 0.0 ? im + 0.5 : im - 0.5);
    return o;
}

/* ---------- 流水线各阶段模块入口（由 ufeq_process 顺序调用） ---------- */

/** 校验 request/result 指针、容量与 config 一致性。 */
ufeq_status_t ufeq_validate_request(const ufeq_request_t *request, const ufeq_result_t *result);

/** AGC 对齐：按天线/符号 AGC 因子缩放 sch_freq_data -> rx_work。 */
ufeq_status_t ufeq_agc_align(const ufeq_param_t *param,
                             const ufeq_data_t *data,
                             ufeq_workspace_t *workspace);

/** DMRS 频偏前校准：估计 delta_theta 并旋转 channel_work。 */
ufeq_status_t ufeq_dmrs_cfo_pre_correct(const ufeq_request_t *request, ufeq_workspace_t *workspace);

/** DMRS 符号间时域线性插值 -> channel_interp。 */
ufeq_status_t ufeq_channel_interpolate(const ufeq_request_t *request, ufeq_workspace_t *workspace);

/** 准备 Ruu：外部注入或残差估计，含对角加载。 */
ufeq_status_t ufeq_prepare_ruu(const ufeq_request_t *request, ufeq_workspace_t *workspace);

/** AUTO 模式下根据天线相关度选择 MRC 或 IRC。 */
ufeq_status_t ufeq_select_equalizer_mode(const ufeq_request_t *request, ufeq_workspace_t *workspace);

/** MMSE 均衡主循环：逐 RE 求 x_hat、rho，写入 equalized_work/sinr_work。 */
ufeq_status_t ufeq_mmse_equalize(const ufeq_request_t *request,
                                 ufeq_workspace_t *workspace,
                                 ufeq_result_t *result);

/** 均衡后频偏校正（post_cfo_phase 或估计相位）。 */
ufeq_status_t ufeq_cfo_post_correct(const ufeq_request_t *request, ufeq_workspace_t *workspace);

/** DFT-s-OFDM 解预编码（IDFT）。 */
ufeq_status_t ufeq_deprecoding(const ufeq_request_t *request, ufeq_workspace_t *workspace);

/** 数据符号残余 CFO 环形估计与补偿。 */
ufeq_status_t ufeq_data_cfo_process(const ufeq_request_t *request, ufeq_workspace_t *workspace);

/** 层逆映射：MIMO 层到码字/输出布局。 */
ufeq_status_t ufeq_layer_demap_process(const ufeq_request_t *request, ufeq_workspace_t *workspace);

/** UCI 第一次解复用：从数据流中抽出 UCI 符号到 result。 */
ufeq_status_t ufeq_uci_demux_first(const ufeq_request_t *request,
                                   ufeq_workspace_t *workspace,
                                   ufeq_result_t *result);

/** 软解调：均衡符号 -> LLR 到 soft_bit_tmp。 */
ufeq_status_t ufeq_demodulate(const ufeq_request_t *request, ufeq_workspace_t *workspace);

/** Gold 序列解扰。 */
ufeq_status_t ufeq_descramble_process(const ufeq_request_t *request, ufeq_workspace_t *workspace);

/** UCI 第二次解复用：ACK/CSI1/CSI2 软比特分流到 result。 */
ufeq_status_t ufeq_uci_demux_second(const ufeq_request_t *request,
                                    ufeq_workspace_t *workspace,
                                    ufeq_result_t *result);

/* ---------- 定点算术工具 ---------- */

/** 有符号 64 位算术右移并四舍五入到 int32。 */
int32_t ufeq_shift_right_round_s32(int64_t value, Uchar shift);

/** 饱和到指定位宽有符号整数（对称）。 */
int32_t ufeq_saturate_s32(int64_t value, Uchar bit_width);

/** int32 饱和到 int16 范围。 */
int16_t ufeq_saturate_s16(int32_t value);

/** int32 饱和到 int8 对称范围 [-127,127]。 */
int8_t ufeq_saturate_s8_sym(int32_t value);

/** 定点复数乘法，输出右移 out_shift 位。 */
ufeq_cint32_t ufeq_complex_mul_q(ufeq_cint32_t a, ufeq_cint32_t b, Uchar out_shift);

/* ---------- 矩阵运算（参考实现，也可被 platform ops 替换部分） ---------- */

/** 通用复数矩阵乘 C = A * B。 */
ufeq_status_t ufeq_matrix_mul(const ufeq_cfloat_t *a,
                              Ushort a_rows,
                              Ushort a_cols,
                              const ufeq_cfloat_t *b,
                              Ushort b_cols,
                              ufeq_cfloat_t *out);

/** 补全 Hermitian 矩阵上三角为共轭对称（就地）。 */
ufeq_status_t ufeq_matrix_hermitian_complete(ufeq_cfloat_t *m, Ushort dim);

/** 参考平台 Hermitian 求逆（Cholesky/高斯消元类实现）。 */
ufeq_status_t ufeq_matrix_inverse_hermitian_ref(const ufeq_cfloat_t *in,
                                                ufeq_cfloat_t *out,
                                                Ushort dim,
                                                Ushort kernel_dim,
                                                float diag_load);

/** 获取 PC 参考平台算子表（libm + 纯 C IDFT）。 */
const ufeq_platform_ops_t *ufeq_platform_ref_ops(void);

#endif /* ufeq_internal_h */
