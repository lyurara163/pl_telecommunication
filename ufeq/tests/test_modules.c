/*
 * UFEQ 模块级单元测试：AGC 对齐与信道插值。
 * 针对独立子模块进行隔离验证，不跑完整 ufeq_process 流水线。
 */
#include "ufeq.h"
#include "ufeq_internal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_fail = 0; /* 累计失败用例数 */

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
 * 测试 AGC 对齐模块。
 * 符号 0 AGC=0（右移 2 位），符号 1 AGC=2（不右移），验证增益补偿是否正确。
 */
static void test_agc(void)
{
    ufeq_config_t cfg;
    ufeq_request_t req;
    ufeq_workspace_t ws;
    Uint bytes = 0;
    void *mem;
    ufeq_sch_freq_data_unit_t *units;
    Schar agc[2];

    ufeq_config_init_default(&cfg);
    cfg.n_rx = 1;
    cfg.n_layer = 1;
    cfg.n_rb = 1;
    cfg.n_sc_rb = 1;
    cfg.m_sc_pusch = 1;
    cfg.n_symb_slot = 2;
    cfg.n_rs = 1;
    cfg.dmrs_symbol[0] = 0;
    cfg.bypass_agc = false; /* 启用 AGC 对齐 */

    units = (ufeq_sch_freq_data_unit_t *)calloc(ufeq_sch_freq_max_symb, sizeof(*units));
    agc[0] = 0;
    agc[1] = 2; /* 符号 1 比符号 0 多 2 级增益，无需右移 */
    test_fill_sch_freq(units, 1, 2, 1, 1, 1024, agc);

    memset(&req, 0, sizeof(req));
    req.param.config = &cfg;
    req.param.start_rb = 0;
    req.data.sch_freq_data = units;

    ufeq_workspace_get_size(&cfg, &bytes);
    mem = malloc(bytes);
    ufeq_workspace_init(&cfg, mem, bytes, &ws);
    check(ufeq_agc_align(&req.param, &req.data, &ws) == ufeq_status_ok, "agc_ok");
    check((int)ws.rx_work[0].re == 256, "agc_shift_sym0"); /* 1024>>2 = 256 */
    check((int)ws.rx_work[1].re == 1024, "agc_no_shift_sym1"); /* AGC 已对齐，保持原值 */
    free(mem);
    free(units);
}

/*
 * 测试 DMRS CFO 预校正与信道时域插值。
 * DMRS 在符号 0/2，验证中间符号 1 的插值结果约为 1.5（h0=1.0, h2=2.0 线性插值）。
 */
static void test_channel_interp(void)
{
    ufeq_config_t cfg;
    ufeq_request_t req;
    ufeq_workspace_t ws;
    Uint bytes = 0;
    void *mem;
    ufeq_cint32_t h[3];
    Ushort i;

    ufeq_config_init_default(&cfg);
    cfg.n_rx = 1;
    cfg.n_layer = 1;
    cfg.n_rb = 1;
    cfg.n_sc_rb = 1;
    cfg.m_sc_pusch = 1;
    cfg.n_symb_slot = 3;
    cfg.n_rs = 2;
    cfg.dmrs_symbol[0] = 0;
    cfg.dmrs_symbol[1] = 2;
    cfg.dmrs_ref_symbol = 0;
    cfg.bypass_dmrs_freq_offset = true;
    cfg.bypass_dmrs_time_filter = false; /* 启用时域插值滤波 */

    for (i = 0; i < 3; ++i) {
        h[i].re = 0;
        h[i].im = 0;
    }
    h[0].re = 32; /* 归一化后 1.0 */
    h[2].re = 64; /* 归一化后 2.0 */

    memset(&req, 0, sizeof(req));
    req.param.config = &cfg;
    req.data.channel_est = h;
    req.data.channel_est_count = 3;
    
    ufeq_workspace_get_size(&cfg, &bytes);
    mem = malloc(bytes);
    ufeq_workspace_init(&cfg, mem, bytes, &ws);
    check(ufeq_dmrs_cfo_pre_correct(&req, &ws) == ufeq_status_ok, "cfo_pre");
    check(ufeq_channel_interpolate(&req, &ws) == ufeq_status_ok, "interp");
    /* 符号 1 位于 0 与 2 中点，插值 ≈ 1.5 */
    check(ws.channel_interp[1].re > 1.4 && ws.channel_interp[1].re < 1.6, "mid_interp");
    free(mem);
}

int main(void)
{
    test_agc();
    test_channel_interp();
    if (g_fail) {
        printf("FAILURES %d\n", g_fail);
        return 1;
    }
    printf("MODULE TESTS PASSED\n");
    return 0;
}
