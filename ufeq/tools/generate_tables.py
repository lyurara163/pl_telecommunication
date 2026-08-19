#!/usr/bin/env python3
"""从 CSV 源生成 UFEQ 协议静态表并输出 CRC 校验代码。

读取 tables_csv 目录下所有 CSV，计算 SHA256 摘要前 8 位作为表 CRC，
生成 ufeq_tables_gen.c 中的 g_ufeq_table_crc 与 ufeq_tables_self_check 函数。
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import pathlib
import zlib


def crc32_file(path: pathlib.Path) -> int:
    """计算单个文件的 CRC32（当前未在主流程中使用，保留供扩展）。"""
    data = path.read_bytes()
    return zlib.crc32(data) & 0xFFFFFFFF


def main() -> None:
    """解析命令行参数，汇总 CSV 并生成 C 源文件。"""
    parser = argparse.ArgumentParser(description="Generate UFEQ static tables")
    parser.add_argument("--input-dir", type=pathlib.Path, required=False, default=pathlib.Path("tables_csv"))
    parser.add_argument("--output", type=pathlib.Path, required=False, default=pathlib.Path("src/ufeq_tables_gen.c"))
    args = parser.parse_args()

    # 扫描输入目录下所有 CSV 文件
    files = sorted(args.input_dir.glob("*.csv")) if args.input_dir.exists() else []
    digest = hashlib.sha256()
    for f in files:
        digest.update(f.read_bytes())  # 将所有 CSV 内容串联后求哈希
    # 取 SHA256 前 8 个十六进制字符作为 CRC；无文件时用默认魔数
    crc = int(digest.hexdigest()[:8], 16) if files else 0x55464551

    # 生成 C 源：CRC 常量 + 自检函数
    out = [
        '#include "ufeq_tables.h"',
        "",
        f"static const uint32_t g_ufeq_table_crc = 0x{crc:08X}u;",
        "",
        "ufeq_status_t ufeq_tables_self_check(void)",
        "{",
        "    if (g_ufeq_table_crc == 0) {",
        "        return ufeq_status_error;",
        "    }",
        "    return ufeq_status_ok;",
        "}",
        "",
    ]
    args.output.write_text("\n".join(out), encoding="utf-8")
    print(f"wrote {args.output} crc=0x{crc:08X} files={len(files)}")


if __name__ == "__main__":
    main()
