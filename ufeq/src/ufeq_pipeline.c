#include "ufeq.h"            /* 对外 API 头文件 */
#include "ufeq_internal.h"   /* 内部头文件：配置/工作区/复数工具等声明 */

#include <stdint.h>          /* 提供 uintptr_t 等整数类型 */
#include <string.h>          /* 提供 memset / memcpy 等内存操作 */

/* 库级日志回调：供无 workspace 上下文的辅助函数（如 ufeq_warn）使用 */
static ufeq_log_callback_t g_ufeq_log_callback = NULL;
static void *g_ufeq_log_user = NULL;

/**
 * 输出告警级别日志（若已注册回调）。
 *
 * 【输入】
 *   module  : 模块名，NULL 时默认为 "ufeq"
 *   message : 告警文本，NULL 时输出空串
 *
 * 【行为】
 *   未注册回调则静默返回；否则经 g_ufeq_log_callback 转发。
 */
void ufeq_warn(const char *module, const char *message)
{
    if (g_ufeq_log_callback == NULL) {
        return; /* 无日志出口，直接返回 */
    }
    g_ufeq_log_callback(ufeq_log_level_warn,
                        (module != NULL) ? module : "ufeq",
                        (message != NULL) ? message : "",
                        g_ufeq_log_user);
}

/**
 * 记录告警并返回通用错误状态。
 *
 * 【输入】
 *   module, message : 同 ufeq_warn
 *
 * 【返回】
 *   ufeq_status_error
 */
ufeq_status_t ufeq_fail(const char *module, const char *message)
{
    ufeq_warn(module, message);       /* 先打日志 */
    return ufeq_status_error;         /* 再返回错误码 */
}

/**
 * 将字节数向上对齐到 64 字节边界。
 *
 * 【目的】
 *   工作区内各缓冲按 64B 对齐，便于 SIMD / 缓存行访问。
 *
 * 【算法】
 *   align64(v) = (v + 63) & ~63
 */
static Uint ufeq_align64(Uint v)
{
    return (v + 63) & ~(Uint)63; /* 掩掉低 6 位，等价于向上取整到 64 的倍数 */
}

/**
 * 从 memory 的 bump 指针区切出一段对齐内存。
 *
 * 【输入】
 *   cursor : 当前 bump 游标（入参/出参）
 *   remain : 剩余可用字节数（入参/出参）
 *   bytes  : 请求分配的字节数
 *   align  : 对齐要求（字节）
 *
 * 【返回】
 *   成功：指向新块的指针；失败（空间不足）：NULL
 *
 * 【算法】
 *   先按 align 填充 padding，再前移 cursor、扣减 remain，不调用 malloc。
 */
static Uchar *ufeq_bump(Uchar **cursor, Uint *remain, Uint bytes, Uint align)
{
    uintptr_t addr = 0;  /* 当前游标地址（整数形式，便于取模） */
    Uint pad = 0;      /* 为满足 align 对齐需跳过的填充字节数 */
    Uchar *ptr = NULL;   /* 返回给调用方的分配起始地址 */

    addr = (uintptr_t)(*cursor);
    pad = (align - (addr % align)) % align; /* 计算到下一对齐边界的 padding */
    if (*remain < pad + bytes) {
        return NULL; /* 剩余空间不足以容纳 padding + 请求块 */
    }
    *cursor += pad;   /* 跳过对齐填充 */
    *remain -= pad;
    ptr = *cursor;     /* 记录对齐后的块起始 */
    *cursor += bytes;  /* 游标前移，占用本块 */
    *remain -= bytes;
    return ptr;
}

/**
 * 将配置结构初始化为 NR PUSCH 均衡器的默认参数。
 *
 * 【目的】
 *   提供开箱即用的最小可行配置，便于单元测试与示例。
 *
 * 【默认要点】
 *   2Rx / 1 层 / 1RB / QPSK / CP-OFDM / MRC / 单 DMRS@符号2
 *
 * 【输入/输出】
 *   config : 待清零并填默认值的配置结构；不可为 NULL
 *
 * 【返回】
 *   ufeq_status_ok 或 invalid_arg
 */
ufeq_status_t ufeq_config_init_default(ufeq_config_t *config)
{
    if (config == NULL) {
        return ufeq_fail("pipeline", "invalid_arg");
    }
    memset(config, 0, sizeof(*config)); /* 全部字段先置零 */
    config->n_rx = 2;                   /* 2 根接收天线 */
    config->n_layer = 1;                /* 1 个 MIMO 层 */
    config->n_rb = 1;                   /* 1 个资源块 */
    config->n_sc_rb = 12;               /* 每 RB 12 个子载波 */
    config->n_symb_slot = 14;           /* 每 slot 14 个 OFDM 符号 */
    config->m_sc_pusch = 12;            /* PUSCH 占用子载波数 = 1RB×12 */
    config->n_mmse_shift = 5;           /* MMSE 输出定点缩放：2^5 */
    config->n_sinr_shift = 11;          /* SINR 输出定点缩放：2^11 */
    config->n_sb_shift = 4;             /* 软比特缩放移位 */
    config->codeword_count = 1;         /* 单码字 */
    config->n_rs = 1;                   /* slot 内 1 个 DMRS 符号 */
    config->dmrs_symbol[0] = 2;         /* DMRS 位于符号 2 */
    config->dmrs_ref_symbol = 2;        /* DMRS 参考符号索引 */
    config->waveform = ufeq_waveform_cp_ofdm; /* CP-OFDM 波形 */
    config->modulation[0] = ufeq_mod_qpsk;    /* 码字 0：QPSK */
    config->equalizer_mode = ufeq_equalizer_mrc; /* 默认 MRC 均衡 */
    config->corr_threshold = 0.3;       /* 均衡模式切换：相关度门限 */
    config->corr_hysteresis = 0.05;   /* 切换迟滞，防抖动 */
    config->ruu_add_coeff = 1e-3;     /* Ruu 对角加载系数 */
    config->ruu_scale = 1.0;          /* Ruu 整体缩放 */
    config->th_in = 0.1;              /* IRC 切入门限 */
    config->th_out = 10.0;            /* IRC 切出门限 */
    config->sinr_threshold_alpha = 256.0; /* SINR 门限平滑系数 */
    config->th_mcs = 100.0;           /* MCS 相关门限 */
    return ufeq_status_ok;
}

/**
 * 按配置估算工作区所需内存总字节数。
 *
 * 【目的】
 *   调用方据此向 ufeq_workspace_init 提供足够大的连续内存块。
 *
 * 【输入】
 *   config    : 均衡配置；不可为 NULL
 *   size_byte : 输出总字节数；不可为 NULL
 *
 * 【算法】
 *   按各中间缓冲元素个数 × sizeof(类型) 累加，每项经 ufeq_align64 对齐，
 *   末尾再加 4096 字节安全余量。
 *
 * 【返回】
 *   ufeq_status_ok 或 invalid_arg
 */
ufeq_status_t ufeq_workspace_get_size(const ufeq_config_t *config, Uint *size_byte)
{
    Uint rx_n = 0;    /* 接收域缓冲元素数：符号×RB×子载波×Rx */
    Uint ch_n = 0;    /* 信道域元素数：rx_n × 层数 */
    Uint eq_n = 0;    /* 均衡域元素数：符号×PUSCH子载波×层数 */
    Uint soft_n = 0;  /* 软比特临时缓冲容量：eq_n × 8（按最大调制阶） */
    Uint total = 0;   /* 累计总字节数 */

    if (config == NULL || size_byte == NULL) {
        return ufeq_fail("pipeline", "invalid_arg");
    }

    rx_n = (Uint)config->n_symb_slot * config->n_rb * config->n_sc_rb * config->n_rx;
    ch_n = rx_n * config->n_layer;
    eq_n = (Uint)config->n_symb_slot * config->m_sc_pusch * config->n_layer;
    soft_n = eq_n * 8; /* 预留 8 倍 eq 长度给软比特/扰码临时区 */

    total = 0;
    total += ufeq_align64(rx_n * sizeof(ufeq_cfloat_t));           /* rx_work */
    total += ufeq_align64(ch_n * sizeof(ufeq_cfloat_t));           /* channel_work */
    total += ufeq_align64(ch_n * sizeof(ufeq_cfloat_t));           /* channel_interp */
    total += ufeq_align64(eq_n * sizeof(ufeq_cfloat_t));           /* equalized_work */
    total += ufeq_align64(eq_n * sizeof(float));                   /* sinr_work */
    total += ufeq_align64(eq_n * sizeof(float));                   /* f_dft */
    total += ufeq_align64((Uint)config->n_symb_slot * config->n_layer * sizeof(float)); /* f_t */
    total += ufeq_align64(eq_n * sizeof(float));                   /* f_cp */
    total += ufeq_align64((Uint)config->n_rx * config->n_rx * sizeof(ufeq_cfloat_t));   /* ruu_work */
    total += ufeq_align64((Uint)config->n_rx * config->n_rx * sizeof(ufeq_cfloat_t));   /* ruu_inv_work */
    total += ufeq_align64((Uint)config->n_layer * config->n_rx * sizeof(ufeq_cfloat_t)); /* h_bar_h */
    total += ufeq_align64((Uint)config->n_layer * config->n_layer * sizeof(ufeq_cfloat_t)); /* r_hh */
    total += ufeq_align64((Uint)config->n_layer * config->n_layer * sizeof(ufeq_cfloat_t)); /* r_hh_inv */
    total += ufeq_align64((Uint)config->n_layer * sizeof(ufeq_cfloat_t)); /* x_tilde */
    total += ufeq_align64((Uint)config->n_layer * sizeof(ufeq_cfloat_t)); /* x_hat */
    total += ufeq_align64((Uint)config->m_sc_pusch * sizeof(ufeq_cfloat_t)); /* idft_tmp */
    total += ufeq_align64(eq_n * sizeof(ufeq_cfloat_t));           /* codeword_symbol[0] */
    total += ufeq_align64(eq_n * sizeof(float));                   /* codeword_sinr[0] */
    total += ufeq_align64(eq_n * sizeof(ufeq_cfloat_t));           /* codeword_symbol[1] */
    total += ufeq_align64(eq_n * sizeof(float));                   /* codeword_sinr[1] */
    total += ufeq_align64(soft_n * sizeof(int8_t));                /* soft_bit_tmp */
    total += ufeq_align64(soft_n * sizeof(Uchar));                 /* scram_tmp */
    total += ufeq_align64(eq_n * sizeof(Uint));                    /* index_buffer */
    total += ufeq_align64(eq_n * sizeof(int8_t));                  /* valid_re_flag */
    total += 4096; /* 额外安全余量，防止对齐累计误差 */
    *size_byte = total;
    return ufeq_status_ok;
}

/**
 * 在外部提供的连续内存上初始化工作区并切分各中间缓冲。
 *
 * 【输入】
 *   config       : 均衡配置
 *   memory       : 调用方预分配的内存块首地址
 *   memory_size  : 内存块字节数，须 ≥ ufeq_workspace_get_size 返回值
 *   workspace    : 待初始化的工作区结构
 *
 * 【算法】
 *   用 ufeq_bump 按 64B 对齐顺序切分 rx/channel/equalize/矩阵/软比特等缓冲，
 *   绑定默认平台 ops，记录 effective 相关计数与软比特偏移初值。
 *
 * 【返回】
 *   ufeq_status_ok、invalid_arg 或 buffer_small
 */
ufeq_status_t ufeq_workspace_init(const ufeq_config_t *config,
                                  void *memory,
                                  Uint memory_size,
                                  ufeq_workspace_t *workspace)
{
    Uint need = 0;    /* 按配置计算的最小所需字节数 */
    Uchar *cursor;      /* bump 分配游标 */
    Uint remain;      /* 剩余可分配字节 */
    Uint rx_n = 0;
    Uint ch_n = 0;
    Uint eq_n = 0;
    Uint soft_n = 0;

    if (config == NULL || memory == NULL || workspace == NULL) {
        return ufeq_fail("pipeline", "invalid_arg");
    }
    if (ufeq_workspace_get_size(config, &need) != ufeq_status_ok || memory_size < need) {
        return ufeq_fail("pipeline", "buffer_small"); /* 外部内存不足 */
    }

    memset(workspace, 0, sizeof(*workspace));
    workspace->config = config;
    workspace->ops = ufeq_platform_ref_ops(); /* 绑定参考平台实现（SIMD 等） */
    workspace->memory = (Uchar *)memory;
    workspace->memory_size = memory_size;
    workspace->selected_mode = config->equalizer_mode; /* 初始均衡模式来自配置 */

    rx_n = (Uint)config->n_symb_slot * config->n_rb * config->n_sc_rb * config->n_rx;
    ch_n = rx_n * config->n_layer;
    eq_n = (Uint)config->n_symb_slot * config->m_sc_pusch * config->n_layer;
    soft_n = eq_n * 8;

    cursor = (Uchar *)memory;
    remain = memory_size;

#define UFEQ_ALLOC(field, type, count)                                                         \
    do {                                                                                       \
        field = (type *)ufeq_bump(&cursor, &remain, sizeof(type) * (Uint)(count), 64);     \
        if ((field) == NULL) {                                                                 \
            return ufeq_fail("pipeline", "buffer_small");                                                   \
        }                                                                                      \
    } while (0)

    UFEQ_ALLOC(workspace->rx_work, ufeq_cfloat_t, rx_n);
    UFEQ_ALLOC(workspace->channel_work, ufeq_cfloat_t, ch_n);
    UFEQ_ALLOC(workspace->channel_interp, ufeq_cfloat_t, ch_n);
    UFEQ_ALLOC(workspace->equalized_work, ufeq_cfloat_t, eq_n);
    UFEQ_ALLOC(workspace->sinr_work, float, eq_n);
    UFEQ_ALLOC(workspace->f_dft, float, eq_n);
    UFEQ_ALLOC(workspace->f_t, float, (Uint)config->n_symb_slot * config->n_layer);
    UFEQ_ALLOC(workspace->f_cp, float, eq_n);
    UFEQ_ALLOC(workspace->ruu_work, ufeq_cfloat_t, (Uint)config->n_rx * config->n_rx);
    UFEQ_ALLOC(workspace->ruu_inv_work, ufeq_cfloat_t, (Uint)config->n_rx * config->n_rx);
    UFEQ_ALLOC(workspace->h_bar_h, ufeq_cfloat_t, (Uint)config->n_layer * config->n_rx);
    UFEQ_ALLOC(workspace->r_hh, ufeq_cfloat_t, (Uint)config->n_layer * config->n_layer);
    UFEQ_ALLOC(workspace->r_hh_inv, ufeq_cfloat_t, (Uint)config->n_layer * config->n_layer);
    UFEQ_ALLOC(workspace->x_tilde, ufeq_cfloat_t, config->n_layer);
    UFEQ_ALLOC(workspace->x_hat, ufeq_cfloat_t, config->n_layer);
    UFEQ_ALLOC(workspace->idft_tmp, ufeq_cfloat_t, config->m_sc_pusch);
    UFEQ_ALLOC(workspace->codeword_symbol[0], ufeq_cfloat_t, eq_n);
    UFEQ_ALLOC(workspace->codeword_sinr[0], float, eq_n);
    UFEQ_ALLOC(workspace->codeword_symbol[1], ufeq_cfloat_t, eq_n);
    UFEQ_ALLOC(workspace->codeword_sinr[1], float, eq_n);
    UFEQ_ALLOC(workspace->soft_bit_tmp, int8_t, soft_n);
    UFEQ_ALLOC(workspace->scram_tmp, Uchar, soft_n);
    UFEQ_ALLOC(workspace->index_buffer, Uint, eq_n);
    UFEQ_ALLOC(workspace->valid_re_flag, int8_t, eq_n);

#undef UFEQ_ALLOC

    workspace->rx_work_count = rx_n;
    workspace->channel_work_count = ch_n;
    workspace->soft_bit_tmp_capacity = soft_n;
    workspace->valid_re_flag_capacity = eq_n;
    workspace->soft_bit_offset[0] = 0; /* 码字 0 软比特写入偏移初值 */
    workspace->soft_bit_offset[1] = 0;
    workspace->soft_bit_count[0] = 0;
    workspace->soft_bit_count[1] = 0;
    return ufeq_status_ok;
}

/**
 * 为工作区替换平台相关运算实现（如 SIMD 内核）。
 *
 * 【输入】
 *   workspace : 已 init 的工作区
 *   ops       : 新的 ufeq_platform_ops 函数表
 */
ufeq_status_t ufeq_workspace_set_platform(ufeq_workspace_t *workspace, const ufeq_platform_ops_t *ops)
{
    if (workspace == NULL || ops == NULL) {
        return ufeq_fail("pipeline", "invalid_arg");
    }
    workspace->ops = ops;
    return ufeq_status_ok;
}

/**
 * 注册 per-workspace 日志回调，并同步为库级告警出口。
 *
 * 【输入】
 *   workspace : 工作区；不可为 NULL
 *   callback  : 日志回调；可为 NULL（表示关闭）
 *   user      : 透传给回调的用户指针
 */
ufeq_status_t ufeq_workspace_set_log(ufeq_workspace_t *workspace,
                                     ufeq_log_callback_t callback,
                                     void *user)
{
    if (workspace == NULL) {
        return ufeq_fail("pipeline", "set_log: null workspace");
    }
    workspace->log_callback = callback;
    workspace->log_user = user;
    /* 同时作为库级告警出口，供无 workspace 的辅助函数使用 */
    g_ufeq_log_callback = callback;
    g_ufeq_log_user = user;
    return ufeq_status_ok;
}

/**
 * 获取工作区内部统计信息指针（只读）。
 *
 * 【返回】
 *   成功：&workspace->stats；workspace 为 NULL 时返回 NULL
 */
const ufeq_stats_t *ufeq_workspace_get_stats(const ufeq_workspace_t *workspace)
{
    if (workspace == NULL) {
        return NULL;
    }
    return &workspace->stats;
}

/**
 * 清零 result 中各输出数组的有效元素计数。
 *
 * 【目的】
 *   每次 ufeq_process 开始前/失败时，避免残留上次的长度字段误导调用方。
 */
static void ufeq_result_clear_counts(ufeq_result_t *result)
{
    result->equalized_count = 0;
    result->sinr_count = 0;
    result->sch_count = 0;
    result->ack_count = 0;
    result->csi1_count = 0;
    result->csi2_count = 0;
    result->uci_syms_count = 0;
    result->uci_sinr_count = 0;
}

/**
 * PUSCH 上行频域均衡主处理流水线（单次 slot 请求）。
 *
 * 【目的】
 *   串联 AGC、CFO、信道插值、Ruu、MMSE、解预编码、解调、解扰、UCI 分离等步骤，
 *   将频域接收数据转为软比特及均衡符号输出。
 *
 * 【输入】
 *   request   : 含 param（配置+DMRS 等）与 data（接收频域样本）
 *   workspace : 预分配的中间缓冲与平台 ops
 *
 * 【输出】
 *   result : 软比特、均衡符号、SINR、UCI、CFO 相位等；失败时 result->status 反映错误码
 *
 * 【流水线顺序】
 *   validate
 *   -> AGC 对齐
 *   -> DMRS 频偏预校正
 *   -> 信道时域插值
 *   -> 估计 Ruu / 选择 MRC|IRC
 *   -> MMSE 均衡
 *   -> 数据 CFO 后校正
 *   -> 解预编码
 *   -> 数据 CFO 处理
 *   -> 层解映射
 *   -> UCI demux1
 *   -> 解调
 *   -> 解扰
 *   -> UCI demux2
 *   -> 将 equalized_work / sinr_work 量化写入 result
 *
 * 【失败策略】
 *   任一步失败则清零 result 计数、设置 result->status 并返回该错误码。
 */
ufeq_status_t ufeq_process(const ufeq_request_t *request,
                           ufeq_result_t *result,
                           ufeq_workspace_t *workspace)
{
    ufeq_status_t st = ufeq_status_ok; /* 当前步骤返回值 */

    if (request == NULL || result == NULL || workspace == NULL) {
        return ufeq_fail("pipeline", "invalid_arg");
    }

    ufeq_result_clear_counts(result);
    result->status = ufeq_status_ok;

    st = ufeq_validate_request(request, result); /* 校验配置与输入缓冲尺寸 */
    if (st != ufeq_status_ok) {
        result->status = st;
        return st;
    }

    /* 有效码字数：配置为 0 时按层数推断（≤4 层→1 码字，否则 2） */
    if (request->param.config->codeword_count == 0) {
        workspace->effective_codeword_count = (request->param.config->n_layer <= 4) ? 1 : 2;
    } else {
        workspace->effective_codeword_count = request->param.config->codeword_count;
    }

    st = ufeq_agc_align(&request->param, &request->data, workspace);
    if (st != ufeq_status_ok) {
        goto fail;
    }
    st = ufeq_dmrs_cfo_pre_correct(request, workspace); /* DMRS 符号频偏预补偿 */
    if (st != ufeq_status_ok) {
        goto fail;
    }
    st = ufeq_channel_interpolate(request, workspace); /* DMRS→全符号信道插值 */
    if (st != ufeq_status_ok) {
        goto fail;
    }
    st = ufeq_prepare_ruu(request, workspace); /* 估计干扰协方差 Ruu */
    if (st != ufeq_status_ok) {
        goto fail;
    }
    st = ufeq_select_equalizer_mode(request, workspace); /* MRC / IRC 模式选择 */
    if (st != ufeq_status_ok) {
        goto fail;
    }
    st = ufeq_mmse_equalize(request, workspace, result); /* MMSE 均衡 + SINR */
    if (st != ufeq_status_ok) {
        goto fail;
    }
    st = ufeq_cfo_post_correct(request, workspace); /* 均衡后 CFO 相位校正 */
    if (st != ufeq_status_ok) {
        goto fail;
    }
    st = ufeq_deprecoding(request, workspace); /* 解预编码 / 层映射逆变换 */
    if (st != ufeq_status_ok) {
        goto fail;
    }
    st = ufeq_data_cfo_process(request, workspace); /* 数据符号 CFO 跟踪/补偿 */
    if (st != ufeq_status_ok) {
        goto fail;
    }
    st = ufeq_layer_demap_process(request, workspace); /* 层→码字解映射 */
    if (st != ufeq_status_ok) {
        goto fail;
    }
    st = ufeq_uci_demux_first(request, workspace, result); /* UCI 第一次分离（解调前） */
    if (st != ufeq_status_ok) {
        goto fail;
    }
    st = ufeq_demodulate(request, workspace); /* 硬/软解调 */
    if (st != ufeq_status_ok) {
        goto fail;
    }
    st = ufeq_descramble_process(request, workspace); /* 解扰 */
    if (st != ufeq_status_ok) {
        goto fail;
    }
    st = ufeq_uci_demux_second(request, workspace, result); /* UCI 第二次分离（解扰后） */
    if (st != ufeq_status_ok) {
        goto fail;
    }

    /* 将浮点 workspace 均衡结果与 SINR 量化为 result 中的定点输出 */
    {
        const ufeq_config_t *cfg = request->param.config;
        float scale_out = (float)(1 << cfg->n_mmse_shift); /* 均衡符号：× 2^n_mmse_shift */
        Uint i;
        for (i = 0; i < workspace->equalized_count; ++i) {
            result->equalized_data[i] = ufeq_cf_to_cint32(workspace->equalized_work[i], scale_out);
            result->sinr[i] =
                (int32_t)(workspace->sinr_work[i] * (float)(1 << cfg->n_sinr_shift) + 0.5); /* 四舍五入 */
        }
        result->equalized_count = workspace->equalized_count;
        result->sinr_count = workspace->equalized_count;
    }

    result->data_cfo_phase = workspace->data_cfo_phase; /* 数据 CFO 估计相位 */
    result->selected_mode = workspace->selected_mode;   /* 实际选用的均衡模式 */
    result->status = ufeq_status_ok;
    return ufeq_status_ok;

fail:
    ufeq_result_clear_counts(result); /* 失败时清空输出长度，防止误用旧数据 */
    result->status = st;
    return st;
}
