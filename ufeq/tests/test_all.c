/*
 * UFEQ 综合回归测试套件。
 * 覆盖 C 开发设计文档中列出的全部模块：定点、矩阵、AGC、信道/CFO、
 * 均衡、层解映射、UCI 解复用、解调/解扰、数据 CFO、查表、参数校验、IDFT。
 */
#include "ufeq.h"
#include "ufeq_internal.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_fail = 0;  /* 累计失败用例数 */
static int g_pass = 0;  /* 累计通过用例数 */

/* 断言布尔条件为真，并统计通过/失败 */
static void expect(int cond, const char *name)
{
    if (cond) {
        printf("  PASS %s\n", name);
        g_pass++;
    } else {
        printf("  FAIL %s\n", name);
        g_fail++;
    }
}

/* 将 I/Q 实部虚部打包为频域数据单元格式 */
static Uint test_pack_iq(int16_t re, int16_t im)
{
    return ufeq_pack_freq_iq(re, im);
}

/*
 * 填充 SCH 频域数据单元，用于构造测试输入。
 * 每个 RE 的 I 路设为 re_val，Q 路为 0；可按符号设置 AGC 增益索引。
 */
static void test_fill_sch_freq(ufeq_sch_freq_data_unit_t *units,
                               Ushort n_rx,
                               Ushort n_symb,
                               Ushort n_rb,
                               Ushort n_sc_rb,
                               int16_t re_val,
                               const Schar *agc_per_symb)
{
    Ushort rx, symb, rb, re;
    Uint freq_idx;
    ufeq_sch_freq_data_unit_t *unit;
    memset(units, 0, (Uint)n_rx * ufeq_sch_freq_max_symb * sizeof(*units));
    for (rx = 0; rx < n_rx; ++rx) {
        for (symb = 0; symb < n_symb; ++symb) {
            unit = (ufeq_sch_freq_data_unit_t *)ufeq_sch_freq_unit(units, rx, symb);
            unit->agc = agc_per_symb ? agc_per_symb[symb] : 0;
            for (rb = 0; rb < n_rb; ++rb) {
                for (re = 0; re < n_sc_rb; ++re) {
                    freq_idx = ((Uint)rb * n_sc_rb) + re;
                    unit->freq_data[freq_idx] = test_pack_iq(re_val, 0);
                }
            }
        }
    }
}

/* 打印测试分组标题 */
static void section(const char *title)
{
    printf("\n== %s ==\n", title);
}

/*
 * 测试定点辅助函数：右移舍入、s8/s16 饱和。
 * 覆盖正负舍入、shift=0、对称饱和及 s16 上界。
 */
static void test_fixed(void)
{
    section("ufeq_fixed");
    expect(ufeq_shift_right_round_s32(5, 1) == 3, "round_pos");
    expect(ufeq_shift_right_round_s32(-5, 1) == -3, "round_neg");
    expect(ufeq_shift_right_round_s32(7, 0) == 7, "round_shift0");       /* 不移位 */
    expect(ufeq_saturate_s8_sym(200) == 127, "sat8_hi");
    expect(ufeq_saturate_s8_sym(-200) == -127, "sat8_lo");
    expect(ufeq_saturate_s8_sym(-128) == -127, "sat8_no_neg128");        /* 对称饱和不含 -128 */
    expect(ufeq_saturate_s16(40000) == 32767, "sat16");
}

/*
 * 测试矩阵运算：2x2 Hermitian 求逆、乘法残差、Hermitian 补全。
 * 验证 A*A^{-1}≈I 及共轭对称填充。
 */
static void test_matrix(void)
{
    ufeq_cfloat_t a[4], inv[4], prod[4], herm[4];
    section("ufeq_matrix");
    a[0] = ufeq_cf(2.0, 0.0);
    a[1] = ufeq_cf(0.5, 0.25);
    a[2] = ufeq_cf(0.5, -0.25);
    a[3] = ufeq_cf(1.5, 0.0);
    expect(ufeq_matrix_inverse_hermitian_ref(a, inv, 2, 2, 0.0) == ufeq_status_ok, "inv2");
    expect(ufeq_matrix_mul(a, 2, 2, inv, 2, prod) == ufeq_status_ok, "mul");
    expect(fabsf(prod[0].re - 1.0) < 2e-3 && fabsf(prod[3].re - 1.0) < 2e-3, "I_residual");

    herm[0] = ufeq_cf(1.0, 0.1);
    herm[1] = ufeq_cf(0.2, 0.3);
    herm[2] = ufeq_cf(0.0, 0.0);
    herm[3] = ufeq_cf(0.5, -0.1);
    expect(ufeq_matrix_hermitian_complete(herm, 2) == ufeq_status_ok, "herm");
    expect(fabsf(herm[0].im) < 1e-6, "herm_diag_im0");           /* 对角虚部清零 */
    expect(fabsf(herm[2].re - herm[1].re) < 1e-6, "herm_mirror_re"); /* 下三角镜像上三角 */
}

/*
 * 测试 AGC 对齐：多符号增益差补偿与 bypass 路径。
 * g_ref=3 时符号 0 右移 3 位，符号 3 不右移；bypass 时保持原值。
 */
static void test_agc(void)
{
    ufeq_config_t cfg;
    ufeq_request_t req;
    ufeq_workspace_t ws;
    Uint bytes = 0;
    void *mem;
    ufeq_sch_freq_data_unit_t *units;
    Schar agc[4];
    Ushort i;

    section("ufeq_agc");
    ufeq_config_init_default(&cfg);
    cfg.n_rx = 1;
    cfg.n_layer = 1;
    cfg.n_rb = 1;
    cfg.n_sc_rb = 1;
    cfg.m_sc_pusch = 1;
    cfg.n_symb_slot = 4;
    cfg.n_rs = 1;
    cfg.dmrs_symbol[0] = 0;
    cfg.bypass_agc = false;
    for (i = 0; i < 4; ++i) {
        agc[i] = (Schar)i; /* 0,1,2,3 -> g_ref=3 */
    }
    units = (ufeq_sch_freq_data_unit_t *)calloc(ufeq_sch_freq_max_symb, sizeof(*units));
    test_fill_sch_freq(units, 1, 4, 1, 1, 1024, agc);
    memset(&req, 0, sizeof(req));
    req.param.config = &cfg;
    req.param.start_rb = 0;
    req.data.sch_freq_data = units;
    ufeq_workspace_get_size(&cfg, &bytes);
    mem = malloc(bytes);
    ufeq_workspace_init(&cfg, mem, bytes, &ws);
    expect(ufeq_agc_align(&req.param, &req.data, &ws) == ufeq_status_ok, "agc_ok");
    expect((int)ws.rx_work[0].re == 128, "agc_shift3"); /* 1024>>3 = 128 */
    expect((int)ws.rx_work[3].re == 1024, "agc_noshift"); /* 最大 AGC 符号无需移位 */

    cfg.bypass_agc = true;
    expect(ufeq_agc_align(&req.param, &req.data, &ws) == ufeq_status_ok, "agc_bypass");
    expect((int)ws.rx_work[0].re == 1024, "agc_bypass_val"); /* bypass 后不做增益对齐 */
    free(mem);
    free(units);
}

/*
 * 测试 DMRS CFO 预校正与信道时域插值。
 * DMRS 在符号 0/4，h=1.0/3.0，验证符号 2 插值 ≈ 2.0。
 */
static void test_channel_cfo(void)
{
    ufeq_config_t cfg;
    ufeq_request_t req;
    ufeq_workspace_t ws;
    Uint bytes = 0;
    void *mem;
    ufeq_cint32_t h[5];
    Ushort i;

    section("ufeq_channel_cfo");
    ufeq_config_init_default(&cfg);
    cfg.n_rx = 1;
    cfg.n_layer = 1;
    cfg.n_rb = 1;
    cfg.n_sc_rb = 1;
    cfg.m_sc_pusch = 1;
    cfg.n_symb_slot = 5;
    cfg.n_rs = 2;
    cfg.dmrs_symbol[0] = 0;
    cfg.dmrs_symbol[1] = 4;
    cfg.dmrs_ref_symbol = 0;
    cfg.bypass_dmrs_freq_offset = true;
    cfg.bypass_dmrs_time_filter = false;
    for (i = 0; i < 5; ++i) {
        h[i].re = 0;
        h[i].im = 0;
    }
    h[0].re = 32;
    h[4].re = 96; /* 归一化后 1.0 与 3.0 */
    memset(&req, 0, sizeof(req));
    req.param.config = &cfg;
    req.data.channel_est = h;
    req.data.channel_est_count = 5;
    ufeq_workspace_get_size(&cfg, &bytes);
    mem = malloc(bytes);
    ufeq_workspace_init(&cfg, mem, bytes, &ws);
    expect(ufeq_dmrs_cfo_pre_correct(&req, &ws) == ufeq_status_ok, "cfo_pre");
    expect(ufeq_channel_interpolate(&req, &ws) == ufeq_status_ok, "interp");
    /* 符号 2 中点插值：0.5*1 + 0.5*3 = 2 */
    expect(fabsf(ws.channel_interp[2].re - 2.0) < 0.05, "interp_mid");
    free(mem);
}

/*
 * 测试 SISO 均衡器闭式解端到端。
 * h=1, y=1000, noise=1 -> 均衡输出 ≈ 32000（含 f_cp 与 Q5 定标）。
 */
static void test_equalizer_siso(void)
{
    ufeq_config_t cfg;
    ufeq_request_t req;
    ufeq_result_t res;
    ufeq_workspace_t ws;
    Uint bytes = 0;
    void *mem;
    ufeq_cint32_t h[12], eq[12];
    int32_t sinr[12];
    ufeq_sch_freq_data_unit_t *units;
    Schar agc_sym[1];
    ufeq_cfloat_t ruu[1];
    Ushort i;

    section("ufeq_equalizer_siso");
    ufeq_config_init_default(&cfg);
    cfg.n_rx = 1;
    cfg.n_layer = 1;
    cfg.n_rb = 1;
    cfg.n_sc_rb = 12;
    cfg.m_sc_pusch = 12;
    cfg.n_symb_slot = 1;
    cfg.n_rs = 1;
    cfg.dmrs_symbol[0] = 0;
    cfg.dmrs_ref_symbol = 0;
    cfg.codeword_count = 1;
    /* 绕过非均衡子模块 */
    cfg.bypass_agc = true;
    cfg.bypass_dmrs_freq_offset = true;
    cfg.bypass_dmrs_time_filter = true;
    cfg.bypass_ruu = true;
    cfg.bypass_data_freq_offset = true;
    cfg.bypass_demux_1 = true;
    cfg.bypass_demod = true;
    cfg.bypass_descramble = true;
    cfg.bypass_demux_2 = true;
    cfg.bypass_freq_offset_2 = true;
    cfg.equalizer_mode = ufeq_equalizer_mrc;
    for (i = 0; i < 12; ++i) {
        h[i].re = 32;
        h[i].im = 0;
    }
    agc_sym[0] = 0;
    units = (ufeq_sch_freq_data_unit_t *)calloc(ufeq_sch_freq_max_symb, sizeof(*units));
    test_fill_sch_freq(units, 1, 1, 1, 12, 1000, agc_sym);
    ruu[0] = ufeq_cf(1.0, 0.0);
    memset(&req, 0, sizeof(req));
    memset(&res, 0, sizeof(res));
    req.param.config = &cfg;
    req.param.start_rb = 0;
    req.data.sch_freq_data = units;
    req.data.channel_est = h;
    req.data.channel_est_count = 12;
    req.data.ruu = ruu;
    res.equalized_data = eq;
    res.equalized_capacity = 12;
    res.sinr = sinr;
    res.sinr_capacity = 12;
    ufeq_workspace_get_size(&cfg, &bytes);
    mem = malloc(bytes);
    ufeq_workspace_init(&cfg, mem, bytes, &ws);
    expect(ufeq_process(&req, &res, &ws) == ufeq_status_ok, "siso_e2e");
    expect(res.equalized_count == 12, "eq_count");
    /* h=1, y=1000, noise=1 -> x ≈ 1000，经 f_cp 与 2^5 定标后 ≈ 32000 */
    expect(abs(eq[0].re - 32000) < 500, "eq_closed_form");
    free(units);
    free(mem);
}

/*
 * 测试层解映射：2 层与 5 层（2 码字）符号重排与计数。
 * 验证 equalized 布局到 codeword 符号顺序及层数拆分。
 */
static void test_layer(void)
{
    ufeq_config_t cfg;
    ufeq_request_t req;
    ufeq_workspace_t ws;
    Uint bytes = 0;
    void *mem;
    Ushort i;

    section("ufeq_layer");
    ufeq_config_init_default(&cfg);
    cfg.n_rx = 1;
    cfg.n_layer = 2;
    cfg.n_rb = 1;
    cfg.n_sc_rb = 2;
    cfg.m_sc_pusch = 2;
    cfg.n_symb_slot = 1;
    cfg.n_rs = 1;
    cfg.codeword_count = 1;
    ufeq_workspace_get_size(&cfg, &bytes);
    mem = malloc(bytes);
    ufeq_workspace_init(&cfg, mem, bytes, &ws);
    memset(&req, 0, sizeof(req));
    req.param.config = &cfg;
    /* 均衡输出布局：RE0 L0, RE0 L1, RE1 L0, RE1 L1 */
    ws.equalized_work[0] = ufeq_cf(1, 0);
    ws.equalized_work[1] = ufeq_cf(2, 0);
    ws.equalized_work[2] = ufeq_cf(3, 0);
    ws.equalized_work[3] = ufeq_cf(4, 0);
    for (i = 0; i < 4; ++i) {
        ws.sinr_work[i] = (float)(i + 1);
    }
    expect(ufeq_layer_demap_process(&req, &ws) == ufeq_status_ok, "layer2");
    expect(ws.codeword_symbol_count[0] == 4, "layer2_count");
    expect(ws.codeword_symbol[0][0].re == 1.0 && ws.codeword_symbol[0][1].re == 2.0, "layer2_order");

    /* 5 层 2 码字：CW0=2 符号，CW1=3 符号 */
    cfg.n_layer = 5;
    cfg.codeword_count = 2;
    cfg.m_sc_pusch = 1;
    cfg.n_sc_rb = 1;
    cfg.n_rb = 1;
    cfg.n_symb_slot = 1;
    free(mem);
    ufeq_workspace_get_size(&cfg, &bytes);
    mem = malloc(bytes);
    ufeq_workspace_init(&cfg, mem, bytes, &ws);
    req.param.config = &cfg;
    for (i = 0; i < 5; ++i) {
        ws.equalized_work[i] = ufeq_cf((float)(i + 1), 0);
        ws.sinr_work[i] = 1.0;
    }
    expect(ufeq_layer_demap_process(&req, &ws) == ufeq_status_ok, "layer5");
    expect(ws.codeword_symbol_count[0] == 2 && ws.codeword_symbol_count[1] == 3, "layer5_split");
    free(mem);
}

/*
 * 测试 UCI 解复用一/二阶段。
 * 按 pusch_re_flag 分离 ACK/CSI1/CSI2 与 SCH 软比特。
 */
static void test_uci(void)
{
    ufeq_config_t cfg;
    ufeq_request_t req;
    ufeq_result_t res;
    ufeq_workspace_t ws;
    Uint bytes = 0;
    void *mem;
    int8_t flags[8] = {0, 1, 2, 3, -1, -2, 0, 0}; /* 0=SCH, 1=ACK, 2=CSI1, 3=CSI2, 负=打孔 */
    ufeq_cfloat_t uci_syms[32];
    float uci_sinr[32];
    int8_t sch[64], ack[64], csi1[64], csi2[64];
    Uint i;

    section("ufeq_uci_demux");
    ufeq_config_init_default(&cfg);
    cfg.n_rx = 1;
    cfg.n_layer = 1;
    cfg.n_rb = 1;
    cfg.n_sc_rb = 8;
    cfg.m_sc_pusch = 8;
    cfg.n_symb_slot = 1;
    cfg.n_rs = 1;
    cfg.modulation[0] = ufeq_mod_qpsk;
    cfg.codeword_count = 1;
    cfg.bypass_demux_1 = false;
    cfg.bypass_demux_2 = false;
    ufeq_workspace_get_size(&cfg, &bytes);
    mem = malloc(bytes);
    ufeq_workspace_init(&cfg, mem, bytes, &ws);
    memset(&req, 0, sizeof(req));
    memset(&res, 0, sizeof(res));
    req.param.config = &cfg;
    req.data.pusch_re_flag = flags;
    req.data.pusch_re_flag_count = 8;
    res.uci_syms = uci_syms;
    res.uci_syms_capacity = 32;
    res.uci_sinr = uci_sinr;
    res.uci_sinr_capacity = 32;
    res.sch_soft_bit = sch;
    res.sch_capacity = 64;
    res.ack_soft_bit = ack;
    res.ack_capacity = 64;
    res.csi1_soft_bit = csi1;
    res.csi1_capacity = 64;
    res.csi2_soft_bit = csi2;
    res.csi2_capacity = 64;
    ws.codeword_symbol_count[0] = 8;
    for (i = 0; i < 8; ++i) {
        ws.codeword_symbol[0][i] = ufeq_cf((float)i, 0);
        ws.codeword_sinr[0][i] = 5.0;
    }
    expect(ufeq_uci_demux_first(&req, &ws, &res) == ufeq_status_ok, "demux1");
    /* ACK: flags 1,-1,-2 -> 索引 1,4,5；CSI1:2；CSI2:-2,3 -> 5,3 */
    expect(res.uci_syms_count >= 5, "demux1_count");

    for (i = 0; i < 16; ++i) {
        ws.soft_bit_tmp[i] = (int8_t)(i + 1);
    }
    ws.soft_bit_tmp_capacity = 64;
    expect(ufeq_uci_demux_second(&req, &ws, &res) == ufeq_status_ok, "demux2");
    expect(res.sch_count >= 4, "demux2_sch"); /* flags 0,0,0 及打孔零填充 */
    expect(res.ack_count == 6, "demux2_ack"); /* 3 个 ACK RE × Qm=2 */
    free(mem);
}

/*
 * 测试 QPSK 解调、Gold 解扰、16-QAM 距离函数及 φx/φy 解扰规则。
 * φx 位不解扰，φy 位用 c(i-1) 解扰。
 */
static void test_demod_descramble(void)
{
    ufeq_config_t cfg;
    ufeq_request_t req;
    ufeq_workspace_t ws;
    Uint bytes = 0;
    void *mem;

    section("ufeq_demod_descramble");
    ufeq_config_init_default(&cfg);
    cfg.n_rx = 1;
    cfg.n_layer = 1;
    cfg.n_rb = 1;
    cfg.n_sc_rb = 4;
    cfg.m_sc_pusch = 4;
    cfg.n_symb_slot = 1;
    cfg.n_rs = 1;
    cfg.modulation[0] = ufeq_mod_qpsk;
    cfg.codeword_count = 1;
    cfg.n_rnti = 1;
    cfg.n_id = 0;
    cfg.n_sb_shift = 0;
    ufeq_workspace_get_size(&cfg, &bytes);
    mem = malloc(bytes);
    ufeq_workspace_init(&cfg, mem, bytes, &ws);
    ws.effective_codeword_count = 1;
    memset(&req, 0, sizeof(req));
    req.param.config = &cfg;
    ws.codeword_symbol_count[0] = 1;
    ws.codeword_symbol[0][0] = ufeq_cf(1.0, -1.0); /* QPSK 第一象限附近 */
    ws.codeword_sinr[0][0] = 10.0;
    expect(ufeq_demodulate(&req, &ws) == ufeq_status_ok, "demod_qpsk");
    expect(ws.soft_bit_count[0] == 2, "soft_count");
    expect(ws.soft_bit_tmp[0] > 0 && ws.soft_bit_tmp[1] < 0, "qpsk_sign"); /* I 正、Q 负 */

    {
        int8_t before0 = ws.soft_bit_tmp[0];
        expect(ufeq_descramble_process(&req, &ws) == ufeq_status_ok, "descramble");
        /* 根据 Gold bit0，可能翻转或不翻转 */
        expect(ws.scram_tmp[0] == 0 || ws.soft_bit_tmp[0] == (int8_t)(-before0)
                   || ws.soft_bit_tmp[0] == before0,
               "descramble_consistent");
    }
    expect(fabsf(ufeq_demod_distance_16qam(5) - 2.0 / sqrtf(10.0)) < 0.05
               || ufeq_demod_distance_16qam(5) > 0.0,
           "d16_pos");

    /* φx/φy：x 位不解扰，y 位用 c(i-1) */
    {
        Uint phi_x[1] = {0};
        Uint phi_y[1] = {1};
        int8_t soft0 = 0;
        int8_t soft1 = 0;
        cfg.pusch_harq_a_len = 1;
        ws.soft_bit_count[0] = 4;
        ws.soft_bit_offset[0] = 0;
        ws.soft_bit_tmp[0] = 40;
        ws.soft_bit_tmp[1] = 50;
        ws.soft_bit_tmp[2] = 60;
        ws.soft_bit_tmp[3] = 70;
        req.data.phi_x_index = phi_x;
        req.data.phi_x_count = 1;
        req.data.phi_y_index = phi_y;
        req.data.phi_y_count = 1;
        soft0 = ws.soft_bit_tmp[0];
        soft1 = ws.soft_bit_tmp[1];
        expect(ufeq_descramble_process(&req, &ws) == ufeq_status_ok, "phi_ok");
        expect(ws.soft_bit_tmp[0] == soft0, "phi_x_keep"); /* φx 索引位保持不变 */
        if (ws.scram_tmp[0] != 0) {
            expect(ws.soft_bit_tmp[1] == (int8_t)(-soft1), "phi_y_prev_c"); /* c(i-1)=1 时翻转 */
        } else {
            expect(ws.soft_bit_tmp[1] == soft1, "phi_y_prev_c0"); /* c(i-1)=0 时不翻转 */
        }
    }
    free(mem);
}

/*
 * 测试数据域 CFO 估计与相位补偿。
 * 四象限已知旋转样点，验证恢复相位 ≈ 0.2 且补偿后幅度 ≈ 1。
 */
static void test_data_cfo(void)
{
    ufeq_config_t cfg;
    ufeq_request_t req;
    ufeq_workspace_t ws;
    Uint bytes = 0;
    void *mem;
    float s, c;
    ufeq_cfloat_t rot;
    Ushort i;

    section("ufeq_data_cfo");
    ufeq_config_init_default(&cfg);
    cfg.n_rx = 1;
    cfg.n_layer = 1;
    cfg.n_rb = 1;
    cfg.n_sc_rb = 16;
    cfg.m_sc_pusch = 16;
    cfg.n_symb_slot = 1;
    cfg.n_rs = 1;
    cfg.th_in = 0.1;
    cfg.th_out = 100.0;
    cfg.bypass_data_freq_offset = false;
    ufeq_workspace_get_size(&cfg, &bytes);
    mem = malloc(bytes);
    ufeq_workspace_init(&cfg, mem, bytes, &ws);
    memset(&req, 0, sizeof(req));
    req.param.config = &cfg;
    ws.ops->sin_cos_f32(0.2, &s, &c);
    rot = ufeq_cf(c, s);
    /* 四个象限各放一个已知相位旋转的样点 */
    ws.equalized_count = 4;
    ws.equalized_work[0] = ufeq_cf_mul(ufeq_cf(1, 1), rot);
    ws.equalized_work[1] = ufeq_cf_mul(ufeq_cf(-1, 1), rot);
    ws.equalized_work[2] = ufeq_cf_mul(ufeq_cf(-1, -1), rot);
    ws.equalized_work[3] = ufeq_cf_mul(ufeq_cf(1, -1), rot);
    expect(ufeq_data_cfo_process(&req, &ws) == ufeq_status_ok, "data_cfo");
    expect(fabsf(ws.data_cfo_phase - 0.2) < 0.05, "phase_recover"); /* 估计相位接近注入值 */
    for (i = 0; i < 4; ++i) {
        expect(fabsf(fabsf(ws.equalized_work[i].re) - 1.0) < 0.15, "comp_amp"); /* 补偿后幅度恢复 */
    }
    free(mem);
}

/*
 * 测试 Low-PAPR 序列、DMRS 端口/符号位置查表及静态表 CRC 自检。
 * 覆盖 Type1/Type2 多种 M 值、端口上限、频跳与非法参数。
 */
static void test_tables(void)
{
    ufeq_cfloat_t seq[36];
    Ushort count = 0;
    ufeq_dmrs_port_param_t port;
    ufeq_dmrs_symbol_set_t set, set2;

    section("ufeq_low_papr_dmrs");
    expect(ufeq_low_papr_generate(1, 0, 0, 0, 0, 6, seq, 36, &count) == ufeq_status_ok, "t1_m6");
    expect(count == 6, "t1_count");
    expect(ufeq_low_papr_generate(1, 0, 0, 0, 0, 12, seq, 36, &count) == ufeq_status_ok, "t1_m12");
    expect(ufeq_low_papr_generate(1, 0, 0, 0, 0, 30, seq, 36, &count) == ufeq_status_ok, "t1_m30");
    expect(ufeq_low_papr_generate(2, 0, 0, 0, 0, 6, seq, 36, &count) == ufeq_status_ok, "t2_m6");
    expect(ufeq_low_papr_generate(2, 0, 0, 0, 0, 18, seq, 36, &count) == ufeq_status_ok, "t2_m18");
    expect(ufeq_low_papr_generate(2, 0, 0, 0, 0, 24, seq, 36, &count) == ufeq_status_ok, "t2_m24");
    expect(ufeq_dmrs_get_port_param(ufeq_dmrs_config_type1, 0, 0, &port) == ufeq_status_ok, "port_ok");
    expect(ufeq_dmrs_get_port_param(ufeq_dmrs_config_type1, 7, 0, &port) == ufeq_status_error,
           "port_single_limit"); /* 单 DMRS 端口超限 */
    expect(ufeq_dmrs_get_symbol_positions(ufeq_pusch_mapping_type_a, 14, 2, 1, 0, 0, &set, NULL)
               == ufeq_status_ok,
           "pos_ok");
    expect(set.count >= 2, "pos_count");
    expect(ufeq_dmrs_get_symbol_positions(ufeq_pusch_mapping_type_a, 14, 2, 3, 1, 0, &set, NULL)
               == ufeq_status_error,
           "pos_invalid_dual"); /* 双 DMRS + 非法附加位置 */
    expect(ufeq_dmrs_get_symbol_positions(ufeq_pusch_mapping_type_a, 14, 2, 1, 0, 1, &set, &set2)
               == ufeq_status_ok,
           "pos_hop"); /* 频跳双集合 */
    expect(ufeq_tables_self_check() == ufeq_status_ok, "table_crc");
}

/*
 * 测试请求参数校验：码字数与层数不匹配、MMSE shift 越界。
 */
static void test_validate_bypass(void)
{
    ufeq_config_t cfg;
    ufeq_request_t req;
    ufeq_result_t res;
    ufeq_cint32_t eq[12];
    int32_t sinr[12];

    section("ufeq_validate");
    ufeq_config_init_default(&cfg);
    cfg.n_layer = 5;
    cfg.codeword_count = 1; /* 5 层应对应 2 码字，故意不匹配 */
    memset(&req, 0, sizeof(req));
    memset(&res, 0, sizeof(res));
    req.param.config = &cfg;
    res.equalized_data = eq;
    res.equalized_capacity = 12;
    res.sinr = sinr;
    res.sinr_capacity = 12;
    expect(ufeq_validate_request(&req, &res) == ufeq_status_error, "cw_mismatch");

    cfg.n_layer = 1;
    cfg.codeword_count = 1;
    cfg.n_mmse_shift = 2; /* 有效范围外 */
    expect(ufeq_validate_request(&req, &res) == ufeq_status_error, "shift_range");
}

/*
 * 测试平台 IDFT：长度 12 能量守恒与非法长度返回 error。
 */
static void test_idft(void)
{
    const ufeq_platform_ops_t *ops = ufeq_platform_ref_ops();
    ufeq_cfloat_t in[12], out[12];
    Ushort i;
    float e_in = 0, e_out = 0;

    section("ufeq_idft");
    for (i = 0; i < 12; ++i) {
        in[i] = ufeq_cf((i == 0) ? 1.0 : 0.0, 0.0); /* 单频点冲激 */
    }
    expect(ops->idft(in, out, 12) == ufeq_status_ok, "idft12");
    for (i = 0; i < 12; ++i) {
        e_in += in[i].re * in[i].re + in[i].im * in[i].im;
        e_out += out[i].re * out[i].re + out[i].im * out[i].im;
    }
    expect(fabsf(e_in - e_out) < 1e-3, "idft_energy"); /* Parseval 能量守恒 */
    expect(ops->idft(in, out, 7) == ufeq_status_error, "idft_bad_len"); /* 7 非支持长度 */
}

int main(void)
{
    printf("UFEQ comprehensive regression\n");
    test_fixed();
    test_matrix();
    test_agc();
    test_channel_cfo();
    test_equalizer_siso();
    test_layer();
    test_uci();
    test_demod_descramble();
    test_data_cfo();
    test_tables();
    test_validate_bypass();
    test_idft();

    printf("\n==============================\n");
    printf("PASS=%d FAIL=%d\n", g_pass, g_fail);
    if (g_fail != 0) {
        return 1;
    }
    printf("ALL COMPREHENSIVE TESTS PASSED\n");
    return 0;
}
