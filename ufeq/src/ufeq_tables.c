#include "ufeq_tables.h"     /* 协议查表对外接口 */
#include "ufeq_internal.h"   /* 内部状态码与失败宏 */

#include <stdint.h>          /* Uint 等定宽整数类型 */

/**
 * 协议表完整性占位校验
 *
 * 【目的】
 *   启动时自检查表数据是否有效。正式发版应由 tools/generate_tables.py
 *   根据官方 CSV 刷新 CRC 常量 g_ufeq_table_crc。
 *
 * 【返回】
 *   g_ufeq_table_crc == 0 → table_crc_error
 *   否则 → ok
 */
static const Uint g_ufeq_table_crc = 0x55464551; /* 占位 CRC（"UFEQ" ASCII 编码） */

ufeq_status_t ufeq_tables_self_check(void)
{
    if (g_ufeq_table_crc == 0) {
        return ufeq_fail("tables", "table_crc_error"); /* CRC 未初始化或损坏 */
    }
    return ufeq_status_ok;
}
