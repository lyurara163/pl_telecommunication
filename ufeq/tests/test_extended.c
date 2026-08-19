/*
 * UFEQ 扩展边界测试：高阶调制解调、Low-PAPR Type2、DMRS 非法参数、矩阵填充求逆。
 * 覆盖常规冒烟/模块测试未触及的边界与异常路径。
 */
#include "ufeq.h"
#include "ufeq_internal.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_fail = 0; /* 累计失败用例数 */

/* 断言布尔条件为真 */
static void check(int cond, const char *name)
{
    if (!cond) {
        printf("FAIL %s\n", name);
        g_fail++;
    } else {
        printf("PASS %s\n", name);
    }
}

/*
 * 测试 16/64/256-QAM 解调在星座原点附近的行为。
 * 验证高阶调制内层 LLR（bit2/bit3）在中心点为正。
 */
static void test_demod_boundaries(void)
{
    ufeq_config_t cfg;
    ufeq_request_t req;
    ufeq_workspace_t ws;
    Uint bytes = 0;
    void *mem;
    ufeq_config_init_default(&cfg);
    cfg.n_symb_slot = 1;
    cfg.n_rb = 1;
    cfg.m_sc_pusch = 12;
    cfg.n_rx = 1;
    cfg.n_layer = 1;
    cfg.n_rs = 1;
    cfg.modulation[0] = ufeq_mod_16qam;
    cfg.bypass_demod = false;
    cfg.codeword_count = 1;
    ufeq_workspace_get_size(&cfg, &bytes);
    mem = malloc(bytes);
    ufeq_workspace_init(&cfg, mem, bytes, &ws);
    memset(&req, 0, sizeof(req));
    req.param.config = &cfg;
    ws.codeword_symbol_count[0] = 1;
    ws.codeword_symbol[0][0] = ufeq_cf(0.0, 0.0); /* 星座原点 */
    ws.codeword_sinr[0][0] = 10.0;
    check(ufeq_demodulate(&req, &ws) == ufeq_status_ok, "demod_16_center");
    /* 原点附近 bit0/1 ≈ 0，bit2/3 应为正（内层判决距离项） */
    check(ws.soft_bit_tmp[2] > 0 && ws.soft_bit_tmp[3] > 0, "demod_16_inner_pos");

    cfg.modulation[0] = ufeq_mod_64qam;
    ws.codeword_symbol[0][0] = ufeq_cf(0.0, 0.0);
    check(ufeq_demodulate(&req, &ws) == ufeq_status_ok, "demod_64_center");

    cfg.modulation[0] = ufeq_mod_256qam;
    check(ufeq_demodulate(&req, &ws) == ufeq_status_ok, "demod_256_center");
    free(mem);
}

/*
 * 测试 Low-PAPR 序列生成：Type2 M=6/12 与 Type1 M=6/30。
 * 验证 Type2 M=6 序列能量接近 1，Type1 输出非零。
 */
static void test_low_papr_type2(void)
{
    ufeq_cfloat_t seq[30];
    Ushort count = 0;
    float energy = 0.0;
    Ushort i;
    check(ufeq_low_papr_generate(2, 0, 0, 0.0, 0, 6, seq, 30, &count) == ufeq_status_ok, "t2_m6");
    check(count == 6, "t2_m6_count");
    for (i = 0; i < count; ++i) {
        energy += seq[i].re * seq[i].re + seq[i].im * seq[i].im;
    }
    check(fabsf(energy / (float)count - 1.0) < 0.15, "t2_m6_energy"); /* 平均能量 ≈ 1 */

    check(ufeq_low_papr_generate(2, 0, 0, 0.0, 0, 12, seq, 30, &count) == ufeq_status_ok, "t2_m12");
    check(ufeq_low_papr_generate(1, 0, 0, 0.0, 0, 6, seq, 30, &count) == ufeq_status_ok, "t1_m6");
    check(fabsf(seq[0].re) > 0.1 || fabsf(seq[0].im) > 0.1, "t1_m6_nonzero"); /* Type1 首样点非零 */
    check(ufeq_low_papr_generate(1, 0, 0, 0.0, 0, 30, seq, 30, &count) == ufeq_status_ok, "t1_m30");
}

/*
 * 测试 DMRS 非法参数与合法参数的分支。
 * 单端口超限、双 DMRS 附加位置非法应返回 error；合法配置应返回 ok。
 */
static void test_dmrs_invalid(void)
{
    ufeq_dmrs_symbol_set_t a, b;
    ufeq_dmrs_port_param_t p;
    check(ufeq_dmrs_get_port_param(ufeq_dmrs_config_type1, 5, 0, &p) == ufeq_status_error,
          "dmrs_single_port_limit"); /* Type1 单 DMRS 端口数上限为 4 */
    check(ufeq_dmrs_get_symbol_positions(ufeq_pusch_mapping_type_a, 14, 2, 3, 1, 0, &a, NULL)
              == ufeq_status_error,
          "dmrs_dual_addpos_invalid"); /* 双 DMRS + 附加位置 3 为非法组合 */
    check(ufeq_dmrs_get_symbol_positions(ufeq_pusch_mapping_type_a, 14, 2, 1, 0, 0, &a, NULL)
              == ufeq_status_ok,
          "dmrs_valid_pos");
    check(a.count >= 2, "dmrs_pos_ge2"); /* 合法配置至少 2 个 DMRS 符号 */
    (void)b;
}

/*
 * 测试 1x1 矩阵填充到 2x2 后的 Hermitian 求逆。
 * 验证 inv[0].re ≈ 0.5（即 1/2）。
 */
static void test_matrix_pad(void)
{
    ufeq_cfloat_t a[1] = {{2.0, 0.0}};
    ufeq_cfloat_t inv[1];
    check(ufeq_matrix_inverse_hermitian_ref(a, inv, 1, 2, 0.0) == ufeq_status_ok, "inv_pad");
    check(fabsf(inv[0].re - 0.5) < 1e-4, "inv_pad_val"); /* 2 的逆为 0.5 */
}

int main(void)
{
    test_demod_boundaries();
    test_low_papr_type2();
    test_dmrs_invalid();
    test_matrix_pad();
    if (g_fail) {
        printf("FAILURES %d\n", g_fail);
        return 1;
    }
    printf("EXTENDED TESTS PASSED\n");
    return 0;
}
