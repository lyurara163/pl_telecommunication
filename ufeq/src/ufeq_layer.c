#include "ufeq_internal.h"   /* 内部头文件：配置/工作区/索引工具等声明 */

#include <string.h>          /* 提供 memcpy 等内存操作函数 */

/**
 * 码字到层的逆映射（3GPP 层映射的逆过程）
 *
 * 【目的】
 *   MMSE 输出是按“层”排列的符号；信道译码/解调通常按“码字”排列。
 *   本模块把层符号还原成码字符号流，SINR 用同一套索引一起搬移。
 *
 * 【规则】
 *   单码字（1~4 层）：按 RE 交错还原
 *     d[n_layer*i + v] = x_v[i]
 *   双码字（5~8 层）：CW0 占前若干层，CW1 占后若干层
 *     典型分割：2+3 / 3+3 / 3+4 / 4+4
 *
 * 【旁路】
 *   bypass_layer_demap=1：仅取 layer 0 符号作为码字 0，不做完整逆映射。
 *
 * 【输出】
 *   workspace->codeword_symbol[cw] ：各码字符号缓冲
 *   workspace->codeword_sinr[cw]   ：各码字 SINR 缓冲
 */
typedef struct {
    Uchar codeword_count; /* 码字数：1 或 2 */
    Uchar layers_cw0;     /* 码字 0 占用的层数 */
    Uchar layers_cw1;     /* 码字 1 占用的层数 */
} ufeq_layer_map_t;

/* 按 n_layer（1~8）查表：g_layer_map[n_layer] 给出码字分割方案 */
static const ufeq_layer_map_t g_layer_map[9] = {
    {0, 0, 0},   /* n_layer=0：非法占位 */
    {1, 1, 0},   /* 1 层 → 1 码字 1 层 */
    {1, 2, 0},   /* 2 层 → 1 码字 2 层 */
    {1, 3, 0},   /* 3 层 → 1 码字 3 层 */
    {1, 4, 0},   /* 4 层 → 1 码字 4 层 */
    {2, 2, 3},   /* 5 层 → 2 码字：2+3 */
    {2, 3, 3},   /* 6 层 → 2 码字：3+3 */
    {2, 3, 4},   /* 7 层 → 2 码字：3+4 */
    {2, 4, 4}    /* 8 层 → 2 码字：4+4 */
};

/**
 * 将均衡后的层符号（及对应 SINR）映射回码字符号缓冲。
 *
 * 【输入布局】
 *   equalized_work[i * n_layer + layer]：第 i 个 RE 的第 layer 层符号
 *
 * 【输出布局】
 *   单码字：按 RE 交错，out = i * layers + local_layer
 *   双码字：CW0 取 layer 0..layers_cw0-1，CW1 取后续层
 */
ufeq_status_t ufeq_layer_demap_process(const ufeq_request_t *request, /* 输入请求（含层数配置） */
                                       ufeq_workspace_t *workspace)   /* 工作区（层→码字映射） */
{
    const ufeq_config_t *cfg = NULL;       /* 指向配置结构 */
    const ufeq_layer_map_t *map = NULL;    /* 当前层数对应的映射表项 */
    Ushort n_layer = 0;                    /* 发送层数 */
    Uint re_count = 0;                   /* RE 总数 = n_symb_slot × m_sc_pusch */
    Uint i = 0;                          /* 当前 RE 索引 */
    Uchar cw = 0;                          /* 当前码字索引 */
    Uchar local_layer = 0;                 /* 码字内的局部层索引 */
    Uchar layers = 0;                      /* 当前码字占用的层数 */
    Uchar layer_base = 0;                  /* 当前码字起始层号 */
    Uint out = 0;                        /* 码字符号输出写指针 */
    Ushort layer = 0;                      /* 全局层索引 */
    Uint src = 0;                        /* equalized_work 源下标 */

    if (request == NULL || workspace == NULL || request->param.config == NULL) {
        return ufeq_fail("layer", "invalid_arg");
    }
    cfg = request->param.config;
    n_layer = cfg->n_layer;
    if (n_layer == 0 || n_layer > 8) {
        return ufeq_fail("layer", "invalid_config");
    }
    map = &g_layer_map[n_layer];
    re_count = (Uint)cfg->n_symb_slot * cfg->m_sc_pusch;

    /* 旁路：仅取 layer 0 作为码字 0，SINR 同步拷贝 */
    if (cfg->bypass_layer_demap) {
        for (i = 0; i < re_count; ++i) {
            workspace->codeword_symbol[0][i] = workspace->equalized_work[i * n_layer];
            workspace->codeword_sinr[0][i] = workspace->sinr_work[i * n_layer];
        }
        workspace->codeword_symbol_count[0] = re_count;
        workspace->codeword_symbol_count[1] = 0;
        return ufeq_status_ok;
    }

    /* 按码字逐个做层→码字逆映射 */
    for (cw = 0; cw < map->codeword_count; ++cw) {
        layers = (cw == 0) ? map->layers_cw0 : map->layers_cw1;
        layer_base = (cw == 0) ? 0 : map->layers_cw0; /* CW1 从 layers_cw0 层开始 */
        out = 0;
        for (i = 0; i < re_count; ++i) {
            for (local_layer = 0; local_layer < layers; ++local_layer) {
                layer = (Ushort)(layer_base + local_layer);
                src = i * n_layer + layer; /* 源：第 i 个 RE 的第 layer 层 */
                workspace->codeword_symbol[cw][out] = workspace->equalized_work[src];
                workspace->codeword_sinr[cw][out] = workspace->sinr_work[src];
                ++out;
            }
        }
        workspace->codeword_symbol_count[cw] = out; /* 记录该码字符号总数 */
    }
    if (map->codeword_count < 2) {
        workspace->codeword_symbol_count[1] = 0; /* 单码字时 CW1 计数清零 */
    }
    return ufeq_status_ok;
}
