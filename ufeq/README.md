# UFEQ 上行频域均衡库（C 参考实现）

基于《UFEQ综合算法设计文档 v2.0》与《UFEQ C语言开发设计文档 v2.0》实现。

## 已完成功能

- AGC 拉齐、DMRS 频偏前校准、信道时域插值/外推
- RUU 计算/保护/修正、MRC/IRC 自适应、Hermitian 求逆（含扩维）
- MMSE 均衡、SINR、匹配滤波、DC 清零、后频偏、IDFT 解预编码
- 数据符号频偏（象限质心）、层逆映射（1～8 层）
- 两次 UCI 解复用、Gold 解扰、软解调（π/2-BPSK～256QAM，按源图分段）
- Low-PAPR Type1（M=6/12/18/24/30/≥36）与 Type2（M=6/12/18/24/≥30，含 IDFT）
- DMRS 端口表 Type1/2 与时域位置查表（单/双符号、频跳、非法组合校验）
- 平台 IP 抽象、定点基础库、workspace 零动态分配流水线

## 测试

```bat
cd ufeq
build_msvc.bat
```

测试套件：
| 可执行文件 | 覆盖 |
|---|---|
| `test_smoke` | 定点、求逆、SISO 端到端、DMRS/Low-PAPR |
| `test_modules` | AGC、信道插值 |
| `test_extended` | 高阶解调、Type2、DMRS 非法组合 |
| `test_all` | 全模块综合回归（68 项） |

当前全部 PASS。

## 顶层 API

```c
ufeq_workspace_get_size(&config, &bytes);
ufeq_workspace_init(&config, memory, bytes, &workspace);
/* request.param = 配置参数；request.data = 输入缓冲 */
request.param.config = &config;
request.param.start_rb = 0;
request.data.sch_freq_data = &gde_sch_freq_data[slot][0][0]; /* [ant][symb] unit */
ufeq_process(&request, &result, &workspace);
```

AGC 入口：`ufeq_agc_align(&request.param, &request.data, &workspace)`，频域数据与 agc 同在 `ufeq_sch_freq_data_unit_t`（16K，对齐平台布局）。

辅助：`ufeq_low_papr_generate`、`ufeq_dmrs_get_*`、`ufeq_demod_distance_*`。

## 工具

- `tools/generate_tables.py`：协议 CSV → CRC
- `tools/vector_convert.py`：黄金浮点向量 → 定点 buffer
- `tools/golden_model.py`：Python 浮点参考（SISO/解调/Gold）

## 后续可选深化

- 定点全路径误差统计与平台 IP 替换（开发文档阶段 2～3）
- 端到端黄金向量库与 BLER 回归

UCI≤2 bit 的 `phi_x`/`phi_y` 占位解扰已按 3GPP 38.211 §6.3.1.1 实现：通过 `request.data.phi_x_index` / `phi_y_index` 传入码字内占位索引。
