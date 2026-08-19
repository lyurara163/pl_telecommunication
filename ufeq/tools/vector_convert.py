#!/usr/bin/env python3
"""将黄金模型浮点向量 JSON 转换为定点二进制请求缓冲。

输入 JSON 格式：{"complex": [{"re": ..., "im": ...}, ...]}
输出：小端 int32 交错 I/Q 的二进制文件，供 C 测试或仿真加载。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import struct


def float_to_q(value: float, shift: int) -> int:
    """浮点转 Q 格式定点整数，四舍五入。"""
    scaled = value * (1 << shift)
    if scaled >= 0:
        return int(scaled + 0.5)
    return int(scaled - 0.5)


def main() -> None:
    """读取 JSON、量化复数样点并写入二进制文件。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("input_json", type=pathlib.Path)
    parser.add_argument("output_bin", type=pathlib.Path)
    parser.add_argument("--shift", type=int, default=5)  # 默认 Q(N,5)
    args = parser.parse_args()

    data = json.loads(args.input_json.read_text(encoding="utf-8"))
    values = data.get("complex", [])
    blobs = bytearray()
    for item in values:
        re = float_to_q(float(item["re"]), args.shift)
        im = float_to_q(float(item["im"]), args.shift)
        blobs += struct.pack("<ii", re, im)  # 小端 32 位 I/Q 交错
    args.output_bin.write_bytes(blobs)
    print(f"wrote {len(values)} complex samples to {args.output_bin}")


if __name__ == "__main__":
    main()
