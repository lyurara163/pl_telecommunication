"""UFEQ 浮点黄金模型：模块级回归参考实现。

提供 SISO MMSE 均衡闭式解、QPSK/16-QAM LLR、信道线性插值、
3GPP Gold 序列等浮点参考，用于与 C 库定点结果对比验证。
"""

from __future__ import annotations

import math
from typing import List, Tuple


def cf_mul(a: complex, b: complex) -> complex:
    """复数乘法（与库内 ufeq_cf_mul 语义一致）。"""
    return a * b


def mmse_siso(y: complex, h: complex, noise: float = 1.0) -> Tuple[complex, float]:
    """SISO MMSE 均衡闭式解，返回 (x_hat * f_cp, rho)。

    与库实现一致：h_bar^H = conj(h)/noise，r_hh = |h|^2/noise + 1，
    rho = |h|^2/noise，f_cp = (1+rho)/rho。
    """
    g = (h.conjugate() / noise) * y
    rhh = (abs(h) ** 2) / noise + 1.0
    x_hat = g / rhh
    rho = 1.0 / (1.0 / rhh) - 1.0  # = rhh - 1
    # Match library: h_bar^H = conj(h)/noise, r_hh = |h|^2/noise + 1
    # rho = 1/r_hh_inv - 1 = r_hh - 1 = |h|^2/noise
    rho = abs(h) ** 2 / noise
    f_cp = (1.0 + rho) / rho if rho > 1e-12 else 1.0
    return x_hat * f_cp, rho


def qpsk_llr(x: complex, rho: float) -> List[float]:
    """QPSK 软比特 LLR：[Re(x)*rho, Im(x)*rho]。"""
    return [x.real * rho, x.imag * rho]


def qam16_llr(x: complex, rho: float) -> List[float]:
    """16-QAM 四比特 LLR：I/Q 外层与内层判决距离。"""
    d = 2.0 / math.sqrt(10.0)  # 16-QAM 归一化内层距离
    return [
        x.real * rho,
        x.imag * rho,
        (d - abs(x.real)) * rho,
        (d - abs(x.imag)) * rho,
    ]


def channel_interp(h_a: complex, h_b: complex, la: int, lb: int, l: int) -> complex:
    """DMRS 符号 la、lb 间对符号 l 的线性信道插值。"""
    if la == lb:
        return h_a
    wa = (lb - l) / (lb - la)  # 靠近 la 的权重
    wb = (l - la) / (lb - la)  # 靠近 lb 的权重
    return wa * h_a + wb * h_b


def gold_bits(c_init: int, length: int) -> List[int]:
    """生成 3GPP Gold 序列比特（x1/x2 LFSR，跳过前 1600 比特）。"""
    x1 = 1
    x2 = c_init & 0x7FFFFFFF
    out = []
    # 1600 比特预热，与协议一致
    for _ in range(1600):
        nb = ((x1 >> 3) ^ x1) & 1
        x1 = (x1 >> 1) | (nb << 30)
        nb = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 1
        x2 = (x2 >> 1) | (nb << 30)
    for _ in range(length):
        out.append((x1 ^ x2) & 1)
        nb = ((x1 >> 3) ^ x1) & 1
        x1 = (x1 >> 1) | (nb << 30)
        nb = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 1
        x2 = (x2 >> 1) | (nb << 30)
    return out


if __name__ == "__main__":
    # 命令行快速自检：SISO、QPSK LLR、Gold 序列
    x, rho = mmse_siso(1000 + 0j, 1 + 0j, 1.0)
    print("siso", x, rho)
    print("qpsk", qpsk_llr(x, rho))
    print("gold", gold_bits(1, 8))
