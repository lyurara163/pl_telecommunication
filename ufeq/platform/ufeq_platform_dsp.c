#include "ufeq_internal.h"   /* ufeq_platform_ops_t、ufeq_platform_ref_ops 声明 */

/**
 * DSP 平台算子入口（占位实现）
 *
 * 【当前行为】
 *   直接回退到 PC 参考算子 ufeq_platform_ref_ops()，保证未集成 DSP 内核时
 *   仍可链接运行、功能与数值路径一致。
 *
 * 【集成指引】
 *   替换本函数体：返回指向 DSP 优化 ufeq_platform_ops_t 的指针，
 *   通常仅需重写 matrix_inverse_hermitian、idft、sin_cos_f32 等热点；
 *   未重写项可继续指向 ref 实现。
 *
 * 【链接】
 *   构建系统通过宏或 CMake 选项在 ref 与 dsp 平台文件间二选一，
 *   或运行时 ufeq_workspace_set_platform 注入。
 */

/** 前向声明：DSP 平台算子表访问入口（本文件定义）。 */
const ufeq_platform_ops_t *ufeq_platform_dsp_ops(void);

/**
 * 获取 DSP 平台算子表。
 * 占位阶段：与 ref 相同；集成后改为 return &g_ufeq_platform_dsp;
 */
const ufeq_platform_ops_t *ufeq_platform_dsp_ops(void)
{
    return ufeq_platform_ref_ops(); /* 占位：委托参考实现 */
}
