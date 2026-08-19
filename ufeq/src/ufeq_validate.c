#include "ufeq_internal.h"   /* 内部头文件：配置常量与状态码 */

/**
 * 请求入参与配置合法性检查
 *
 * 【何时调用】
 *   ufeq_process 入口处最先执行。任一项不通过则整条流水线不启动。
 *
 * 【检查重点】
 *   1) 天线数 / 层数 / RB / 符号数落在库支持的上限内
 *   2) m_sc_pusch 必须等于 n_rb * n_sc_rb
 *   3) 定标移位：n_mmse_shift∈[3,6]，n_sinr_shift∈[10,12]
 *   4) DMRS 符号个数 n_rs 合法
 *   5) 码字数与层数匹配（1~4层→1CW，5~8层→2CW；0 表示运行时推断）
 *   6) 非 bypass 时 sch_freq_data / channel_est 容量够用
 *   7) result 的 equalized / sinr 输出缓冲容量够用
 */
ufeq_status_t ufeq_validate_request(const ufeq_request_t *request, /* 输入请求 */
                                    const ufeq_result_t *result)   /* 输出结果缓冲（容量检查） */
{
    const ufeq_config_t *cfg = NULL; /* 配置指针 */
    Uint rx_need = 0;              /* 接收频域数据元素数（预留） */
    Uint ch_need = 0;              /* 信道估计元素数 */
    Uint eq_need = 0;              /* 均衡输出元素数 */
    Uchar expect_cw = 0;             /* 层数对应的期望码字数 */

    /* 空指针检查 */
    if (request == NULL || result == NULL || request->param.config == NULL) {
        return ufeq_fail("validate", "invalid_arg");
    }
    cfg = request->param.config;

    /* 接收天线数：1 ~ ufeq_max_rx_ant */
    if (cfg->n_rx == 0 || cfg->n_rx > ufeq_max_rx_ant) {
        return ufeq_fail("validate", "invalid_config");
    }
    /* 发送层数：1 ~ ufeq_max_layer */
    if (cfg->n_layer == 0 || cfg->n_layer > ufeq_max_layer) {
        return ufeq_fail("validate", "invalid_config");
    }
    /* RB / 每RB子载波 / slot 符号数合法性 */
    if (cfg->n_rb == 0 || cfg->n_sc_rb == 0 || cfg->n_symb_slot == 0
        || cfg->n_symb_slot > ufeq_max_symb_slot) {
        return ufeq_fail("validate", "invalid_config");
    }
    /* PUSCH 子载波数须等于 RB 数 × 每 RB 子载波数 */
    if (cfg->m_sc_pusch != (Ushort)(cfg->n_rb * cfg->n_sc_rb)) {
        return ufeq_fail("validate", "invalid_config");
    }
    /* MMSE 定标移位范围 [3, 6] */
    if (cfg->n_mmse_shift < 3 || cfg->n_mmse_shift > 6) {
        return ufeq_fail("validate", "invalid_config");
    }
    /* SINR 定标移位范围 [10, 12] */
    if (cfg->n_sinr_shift < 10 || cfg->n_sinr_shift > 12) {
        return ufeq_fail("validate", "invalid_config");
    }
    /* DMRS 符号个数：1 ~ ufeq_max_dmrs_symb */
    if (cfg->n_rs == 0 || cfg->n_rs > ufeq_max_dmrs_symb) {
        return ufeq_fail("validate", "invalid_config");
    }

    /* 码字数与层数匹配：≤4 层 → 1 CW，>4 层 → 2 CW */
    expect_cw = (cfg->n_layer <= 4) ? 1 : 2;
    if (cfg->codeword_count == 0) {
        /* codeword_count=0 允许，运行时按 expect_cw 推断；此处不拒绝 */
    } else if (cfg->codeword_count != expect_cw && cfg->n_layer <= 8) {
        return ufeq_fail("validate", "invalid_config");
    }

    /* 计算各缓冲所需元素个数 */
    rx_need = (Uint)cfg->n_symb_slot * cfg->n_rb * cfg->n_sc_rb * cfg->n_rx;
    (void)rx_need;
    ch_need = (Uint)cfg->n_symb_slot * cfg->n_rb * cfg->n_sc_rb * cfg->n_rx * cfg->n_layer;
    eq_need = (Uint)cfg->n_symb_slot * cfg->m_sc_pusch * cfg->n_layer;

    /* 非旁路符号处理：检查频域数据与信道估计 */
    if (!cfg->bypass_symbol_process) {
        if (request->data.sch_freq_data == NULL) {
            return ufeq_fail("validate", "invalid_arg");
        }
        /* start_rb + n_rb 不得超出频域缓冲 RB 上限 */
        if ((Ushort)(request->param.start_rb + cfg->n_rb) > ufeq_sch_freq_max_rb) {
            return ufeq_fail("validate", "invalid_arg");
        }
        if (request->data.channel_est == NULL || request->data.channel_est_count < ch_need) {
            return ufeq_fail("validate", "invalid_arg");
        }
    } else {
        /* 旁路模式：检查预均衡输入容量 */
        if (request->data.pre_equalized == NULL || request->data.pre_equalized_count < eq_need) {
            return ufeq_fail("validate", "invalid_arg");
        }
    }

    /* 输出缓冲容量检查：均衡符号与 SINR */
    if (result->equalized_data == NULL || result->equalized_capacity < eq_need) {
        return ufeq_fail("validate", "buffer_small");
    }
    if (result->sinr == NULL || result->sinr_capacity < eq_need) {
        return ufeq_fail("validate", "buffer_small");
    }
    return ufeq_status_ok;
}
