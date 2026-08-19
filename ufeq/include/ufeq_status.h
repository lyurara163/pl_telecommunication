#ifndef ufeq_status_h
#define ufeq_status_h

/**
 * UFEQ 统一函数返回状态。
 *
 * 【约定】
 *   库内可返回 void 或本枚举的 ok / error 两档。
 *   细分失败原因（如 invalid_arg、buffer_small）不编码进返回值，
 *   通过 ufeq_warn / ufeq_fail + set_log 回调输出，便于联调又不膨胀 API。
 *
 * 【用法】
 *   成功：return ufeq_status_ok;
 *   失败：return ufeq_fail("module", "reason");（内部会打 warn 并返回 error）
 */
typedef enum {
    ufeq_status_ok = 0,    /* 调用成功完成 */
    ufeq_status_error = -1 /* 参数/缓冲/数值/不支持等任一失败 */
} ufeq_status_t;

#endif /* ufeq_status_h */
