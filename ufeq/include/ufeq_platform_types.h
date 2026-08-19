#ifndef ufeq_platform_types_h
#define ufeq_platform_types_h

/**
 * DSP 平台类型适配头（仅在定义 UFEQ_USE_PLATFORM_TYPES 时包含）。
 *
 * 【用途】
 *   将 UFEQ 源码中的 Uchar/Ushort/Uint/Schar/Sint 映射到目标 DSP/SoC
 *   原生类型，避免在算法 .c 中散落 #ifdef。
 *
 * 【集成步骤】
 *   1. 编译时定义 UFEQ_USE_PLATFORM_TYPES
 *   2. 在本文件顶部 #include 你们平台的类型头，例如：
 *        #include "xxxx_types.h"
 *   3. 确认平台头已定义 Uchar、Ushort、Uint、Schar、Sint
 *
 * 若平台头文件名不同，只需改本文件，无需改算法源码。
 */

/* 集成示例（取消注释并改为实际路径）：
 * #include "your_platform_types.h"
 */

/* ---------- 编译期检查：平台类型必须已由上方 include 提供 ---------- */

#ifndef Uchar
#error "UFEQ_USE_PLATFORM_TYPES is set but Uchar is not defined. Include your platform types header above."
#endif
#ifndef Ushort
#error "UFEQ_USE_PLATFORM_TYPES is set but Ushort is not defined. Include your platform types header above."
#endif
#ifndef Uint
#error "UFEQ_USE_PLATFORM_TYPES is set but Uint is not defined. Include your platform types header above."
#endif

#ifndef Schar
#error "UFEQ_USE_PLATFORM_TYPES is set but Schar is not defined. Include your platform types header above."
#endif
#ifndef Sint
#error "UFEQ_USE_PLATFORM_TYPES is set but Sint is not defined. Include your platform types header above."
#endif

#endif /* ufeq_platform_types_h */
