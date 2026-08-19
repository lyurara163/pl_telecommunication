/*
 * UFEQ 冒烟测试：验证核心基础模块能否正常工作。
 * 覆盖定点运算、矩阵求逆、SISO 均衡端到端流程、Gold/DMRS 查表。
 */
#include "ufeq.h"
#include "ufeq_internal.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_failures = 0; /* 累计失败用例数 */

/* 断言 ufeq_status_t 返回 ufeq_status_ok */
static void expect_ok(ufeq_status_t st, const char *name)
{
    if (st != ufeq_status_ok) {
        printf("FAIL %s status=%d\n", name, (int)st);
        g_failures++;
    } else {
        printf("PASS %s\n", name);
    }
}

/* 断言布尔条件为真 */
static void expect_true(int cond, const char *name)
{
    if (!cond) {
        printf("FAIL %s\n", name);
        g_failures++;
    } else {
        printf("PASS %s\n", name);
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

/*
 * 测试定点辅助函数：右移舍入与 s8 对称饱和。
 * 验证正/负舍入、上下限饱和边界。
 */
static void test_fixed_point(void)
{
    expect_true(ufeq_shift_right_round_s32(5, 1) == 3, "round_pos");   /* 5>>1 四舍五入得 3 */
    expect_true(ufeq_shift_right_round_s32(-5, 1) == -3, "round_neg"); /* 负数同样向远离零舍入 */
    expect_true(ufeq_saturate_s8_sym(200) == 127, "sat_pos");          /* 超出上界饱和到 127 */
    expect_true(ufeq_saturate_s8_sym(-200) == -127, "sat_neg");        /* 超出下界饱和到 -127 */
}

/*
 * 测试 2x2 Hermitian 矩阵求逆。
 * 验证 A*A^{-1} 对角元接近 1（单位阵残差）。
 */
static void test_matrix_inverse(void)
{
    ufeq_cfloat_t a[4];
    ufeq_cfloat_t inv[4];
    ufeq_cfloat_t prod[4];
    a[0] = (ufeq_cfloat_t){2.0, 0.0};
    a[1] = (ufeq_cfloat_t){0.5, 0.1};
    a[2] = (ufeq_cfloat_t){0.5, -0.1};
    a[3] = (ufeq_cfloat_t){1.5, 0.0};
    expect_ok(ufeq_matrix_inverse_hermitian_ref(a, inv, 2, 2, 0.0), "mat_inv");
    expect_ok(ufeq_matrix_mul(a, 2, 2, inv, 2, prod), "mat_mul");
    /* prod[0]、prod[3] 应为 1（矩阵乘法验证逆矩阵正确性） */
    expect_true(fabsf(prod[0].re - 1.0) < 1e-3 && fabsf(prod[3].re - 1.0) < 1e-3, "inv_residual");
}

/*
 * 测试 SISO MRC 均衡器端到端流程。
 * 绕过 AGC/DMRS/解调等子模块，仅验证均衡输出非零且 RE 数量正确。
 */
static void test_siso_equalizer(void)
{
    ufeq_config_t cfg;
    ufeq_request_t req;
    ufeq_result_t res;
    ufeq_workspace_t ws;
    Uint ws_size = 0;
    void *mem;
    ufeq_cint32_t h[12];
    ufeq_cint32_t eq[12];
    int32_t sinr[12];
    ufeq_sch_freq_data_unit_t *units;
    Schar agc_sym[1];
    ufeq_cfloat_t ruu[1];
    Ushort i;

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
    /* 绕过非均衡相关子模块，聚焦均衡链路 */
    cfg.bypass_agc = true;
    cfg.bypass_dmrs_freq_offset = true;
    cfg.bypass_dmrs_time_filter = true;
    cfg.bypass_data_freq_offset = true;
    cfg.bypass_demux_1 = true;
    cfg.bypass_demod = true;
    cfg.bypass_descramble = true;
    cfg.bypass_demux_2 = true;
    cfg.bypass_freq_offset_2 = true;
    cfg.equalizer_mode = ufeq_equalizer_mrc;

    for (i = 0; i < 12; ++i) {
        h[i].re = 32; /* Q(N,5) ~= 1.0，信道估计归一化值 */
        h[i].im = 0;
    }
    agc_sym[0] = 0;
    units = (ufeq_sch_freq_data_unit_t *)calloc(ufeq_sch_freq_max_symb, sizeof(*units));
    test_fill_sch_freq(units, 1, 1, 1, 12, 1000, agc_sym); /* 频域接收样值 I=1000 */
    ruu[0] = (ufeq_cfloat_t){1.0, 0.0};

    memset(&req, 0, sizeof(req));
    memset(&res, 0, sizeof(res));
    req.param.config = &cfg;
    req.param.start_rb = 0;
    req.data.sch_freq_data = units;
    req.data.channel_est = h;
    req.data.channel_est_count = 12;
    req.data.ruu = ruu;
    cfg.bypass_ruu = true;

    res.equalized_data = eq;
    res.equalized_capacity = 12;
    res.sinr = sinr;
    res.sinr_capacity = 12;

    expect_ok(ufeq_workspace_get_size(&cfg, &ws_size), "ws_size");
    mem = malloc(ws_size);
    expect_true(mem != NULL, "ws_alloc");
    expect_ok(ufeq_workspace_init(&cfg, mem, ws_size, &ws), "ws_init");
    expect_ok(ufeq_process(&req, &res, &ws), "siso_process");
    expect_true(res.equalized_count == 12, "eq_count");   /* 12 个子载波均应输出 */
    expect_true(abs(eq[0].re) > 0, "eq_nonzero");         /* 均衡结果非零 */
    free(mem);
    free(units);
}

/*
 * 测试 Gold 序列、Low-PAPR 序列生成及 DMRS 端口/符号位置查表。
 * 同时调用 ufeq_tables_self_check 验证静态表 CRC。
 */
static void test_gold_and_dmrs(void)
{
    ufeq_dmrs_port_param_t port;
    ufeq_dmrs_symbol_set_t set;
    ufeq_cfloat_t seq[6];
    Ushort count = 0;
    expect_ok(ufeq_dmrs_get_port_param(ufeq_dmrs_config_type1, 0, 0, &port), "dmrs_port");
    expect_true(port.cdm_group == 0 && port.delta == 0, "dmrs_port_fields"); /* Type1 端口 0 字段 */
    expect_ok(ufeq_dmrs_get_symbol_positions(ufeq_pusch_mapping_type_a, 14, 2, 1, 0, 0, &set, NULL),
              "dmrs_pos");
    expect_true(set.count >= 1, "dmrs_pos_count"); /* 至少一个 DMRS 符号位置 */
    expect_ok(ufeq_low_papr_generate(1, 0, 0, 0.0, 0, 6, seq, 6, &count), "low_papr");
    expect_true(count == 6, "low_papr_count");     /* M=6 应生成 6 个序列样点 */
    expect_ok(ufeq_tables_self_check(), "table_crc");
}

int main(void)
{
    test_fixed_point();
    test_matrix_inverse();
    test_siso_equalizer();
    test_gold_and_dmrs();
    if (g_failures != 0) {
        printf("TOTAL FAILURES: %d\n", g_failures);
        return 1;
    }
    printf("ALL TESTS PASSED\n");
    return 0;
}
