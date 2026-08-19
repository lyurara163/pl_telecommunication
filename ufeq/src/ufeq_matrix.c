#include "ufeq_internal.h"   /* 内部头文件：复数运算、常量与状态码 */

#include <math.h>            /* sqrtf / fabsf / isfinite 等数学函数 */
#include <string.h>          /* memcpy 等内存操作 */

/**
 * Hermitian 正定矩阵 Cholesky 求逆（参考实现）
 *
 * 【目的】
 *   对 Hermitian 正定矩阵 A 做 Cholesky 分解 A = L L^H，再通过前/回代求 A^{-1}。
 *
 * 【输入/输出】
 *   a_in    : 输入 n×n Hermitian 矩阵（行主序）
 *   a_inv   : 输出 n×n 逆矩阵
 *   n       : 矩阵维数
 *   diag_load : 对角加载量，提高病态矩阵数值稳定性
 *
 * 【算法要点】
 *   1) 拷贝输入并在对角加 diag_load
 *   2) 经典 Cholesky 分解得到下三角 L
 *   3) 对单位阵各列做 L y = e 前代，再 L^H x = y 回代得逆矩阵列
 *   4) 对角过小（≤ ufeq_diag_protect）或逆矩阵非正定 → 返回 singular
 */
static ufeq_status_t ufeq_cholesky_inverse(const ufeq_cfloat_t *a_in, /* 输入 Hermitian 矩阵 */
                                           ufeq_cfloat_t *a_inv,       /* 输出逆矩阵缓冲 */
                                           Ushort n,                   /* 矩阵维数 */
                                           float diag_load)            /* 对角加载量 */
{
    ufeq_cfloat_t a[ufeq_max_rx_ant * ufeq_max_rx_ant] = {0}; /* 工作副本（含对角加载） */
    ufeq_cfloat_t l[ufeq_max_rx_ant * ufeq_max_rx_ant] = {0}; /* Cholesky 下三角因子 L */
    ufeq_cfloat_t y[ufeq_max_rx_ant] = {0};                   /* 前代中间向量 */
    Ushort i = 0;        /* 行索引 */
    Ushort j = 0;        /* 列索引 */
    Ushort k = 0;        /* 内积累加索引 */
    Ushort col = 0;      /* 当前求逆的列号（对应单位阵列） */
    ufeq_cfloat_t sum = {0}; /* 内积累加和 */
    float diag = 0.0;    /* 对角元素实部（须为正） */
    float den = 0.0;     /* 非对角归一化分母 L[j][j] */
    ufeq_cfloat_t num = {0}; /* 非对角分子 A[i][j] - sum */

    /* 维数合法性检查 */
    if (n == 0 || n > ufeq_max_rx_ant) {
        return ufeq_fail("matrix", "invalid_arg");
    }

    /* 拷贝输入矩阵到本地工作区 */
    memcpy(a, a_in, sizeof(ufeq_cfloat_t) * (Uint)n * (Uint)n);
    /* 对角加载：a_ii += diag_load，改善条件数 */
    for (i = 0; i < n; ++i) {
        a[i * n + i].re += diag_load;
    }

    /* Cholesky 分解：A = L L^H，L 为下三角 */
    for (i = 0; i < n; ++i) {
        for (j = 0; j <= i; ++j) {
            sum = ufeq_cf(0.0, 0.0);
            /* sum = Σ_k L[i][k] * conj(L[j][k])，k < j */
            for (k = 0; k < j; ++k) {
                sum = ufeq_cf_add(sum, ufeq_cf_mul(l[i * n + k], ufeq_cf_conj(l[j * n + k])));
            }
            if (i == j) {
                /* 对角：L[i][i] = sqrt(A[i][i] - sum) */
                diag = a[i * n + i].re - sum.re;
                if (diag <= ufeq_diag_protect) {
                    return ufeq_fail("matrix", "matrix_singular"); /* 非正定，分解失败 */
                }
                l[i * n + j] = ufeq_cf(sqrtf(diag), 0.0);
            } else {
                /* 非对角：L[i][j] = (A[i][j] - sum) / L[j][j] */
                den = l[j * n + j].re;
                if (fabsf(den) < ufeq_diag_protect) {
                    return ufeq_fail("matrix", "matrix_singular");
                }
                num = ufeq_cf_sub(a[i * n + j], sum);
                l[i * n + j] = ufeq_cf_scale(num, 1.0 / den);
            }
        }
    }

    /* 逐列求逆：解 L L^H x = e_col */
    for (col = 0; col < n; ++col) {
        /* 前代：L y = e_col */
        for (i = 0; i < n; ++i) {
            sum = (i == col) ? ufeq_cf(1.0, 0.0) : ufeq_cf(0.0, 0.0); /* 单位阵第 col 列 */
            for (k = 0; k < i; ++k) {
                sum = ufeq_cf_sub(sum, ufeq_cf_mul(l[i * n + k], y[k]));
            }
            y[i] = ufeq_cf_scale(sum, 1.0 / l[i * n + i].re);
        }
        /* 回代：L^H x = y，结果写入 a_inv 第 col 列 */
        for (i = n; i-- > 0;) {
            sum = y[i];
            for (k = (Ushort)(i + 1); k < n; ++k) {
                sum = ufeq_cf_sub(sum, ufeq_cf_mul(ufeq_cf_conj(l[k * n + i]), a_inv[k * n + col]));
            }
            a_inv[i * n + col] = ufeq_cf_scale(sum, 1.0 / l[i * n + i].re);
        }
    }

    /* 校验逆矩阵对角：须有限且为正实 */
    for (i = 0; i < n; ++i) {
        if (!(isfinite(a_inv[i * n + i].re)) || a_inv[i * n + i].re <= 0.0) {
            return ufeq_fail("matrix", "matrix_singular");
        }
    }
    return ufeq_status_ok;
}

/**
 * 复数稠密矩阵乘法 C = A * B（行主序）
 *
 * 【输入/输出】
 *   a       : A 矩阵，维度 a_rows × a_cols
 *   b       : B 矩阵，维度 a_cols × b_cols
 *   out     : 输出 C，维度 a_rows × b_cols
 *
 * 【算法】
 *   C[i][j] = Σ_k A[i][k] * B[k][j]
 */
ufeq_status_t ufeq_matrix_mul(const ufeq_cfloat_t *a, /* 左矩阵 A */
                              Ushort a_rows,          /* A 行数 */
                              Ushort a_cols,          /* A 列数 / B 行数 */
                              const ufeq_cfloat_t *b, /* 右矩阵 B */
                              Ushort b_cols,          /* B 列数 */
                              ufeq_cfloat_t *out)     /* 输出 C */
{
    Ushort i = 0;              /* 输出行索引 */
    Ushort j = 0;              /* 输出列索引 */
    Ushort k = 0;              /* 内积累加索引 */
    ufeq_cfloat_t sum = {0};   /* 单元素累加和 */

    /* 空指针与零维检查 */
    if (a == NULL || b == NULL || out == NULL || a_rows == 0 || a_cols == 0 || b_cols == 0) {
        return ufeq_fail("matrix", "invalid_arg");
    }
    for (i = 0; i < a_rows; ++i) {
        for (j = 0; j < b_cols; ++j) {
            sum = ufeq_cf(0.0, 0.0);
            for (k = 0; k < a_cols; ++k) {
                /* C[i][j] += A[i][k] * B[k][j] */
                sum = ufeq_cf_add(sum, ufeq_cf_mul(a[i * a_cols + k], b[k * b_cols + j]));
            }
            out[i * b_cols + j] = sum;
        }
    }
    return ufeq_status_ok;
}

/**
 * Hermitian 矩阵补全（上三角 → 全矩阵）
 *
 * 【目的】
 *   只填了上三角的 Hermitian 矩阵，用共轭对称补全下三角：
 *     m[j][i] = conj(m[i][j])，i < j
 *
 * 【后处理】
 *   对角强制为实数，且不小于 ufeq_diag_protect（用于 Ruu 等半填矩阵）
 */
ufeq_status_t ufeq_matrix_hermitian_complete(ufeq_cfloat_t *m, /* 待补全矩阵（原地修改） */
                                               Ushort dim)       /* 矩阵维数 */
{
    Ushort i = 0; /* 行索引 */
    Ushort j = 0; /* 列索引 */

    if (m == NULL || dim == 0) {
        return ufeq_fail("matrix", "invalid_arg");
    }
    for (i = 0; i < dim; ++i) {
        /* 对角保护：实部下限 + 虚部清零 */
        if (m[i * dim + i].re < ufeq_diag_protect) {
            m[i * dim + i].re = ufeq_diag_protect;
        }
        m[i * dim + i].im = 0.0;
        /* 下三角 = 上三角共轭：m[j][i] = conj(m[i][j]) */
        for (j = i + 1; j < dim; ++j) {
            m[j * dim + i] = ufeq_cf_conj(m[i * dim + j]);
        }
    }
    return ufeq_status_ok;
}

/**
 * 平台可替换的 Hermitian 求逆入口（参考实现走 Cholesky）
 *
 * 【目的】
 *   对 dim×dim Hermitian 矩阵求逆；若 kernel_dim > dim，先零填充/单位阵扩展到 kernel_dim 再求逆。
 *
 * 【输入/输出】
 *   in         : 输入矩阵（dim×dim）
 *   out        : 输出逆矩阵（dim×dim，从 padded 结果截取）
 *   dim        : 有效数据维数
 *   kernel_dim : 内核运算维数（≥ dim，用于对齐硬件核）
 *   diag_load  : Cholesky 对角加载；首次失败会加大 1e-3 重试
 */
ufeq_status_t ufeq_matrix_inverse_hermitian_ref(const ufeq_cfloat_t *in, /* 输入 Hermitian 矩阵 */
                                                ufeq_cfloat_t *out,      /* 输出逆矩阵 */
                                                Ushort dim,              /* 有效维数 */
                                                Ushort kernel_dim,       /* 内核维数（可大于 dim） */
                                                float diag_load)         /* 对角加载量 */
{
    ufeq_cfloat_t padded[ufeq_max_rx_ant * ufeq_max_rx_ant] = {0};     /* 扩展后的输入 */
    ufeq_cfloat_t padded_inv[ufeq_max_rx_ant * ufeq_max_rx_ant] = {0}; /* 扩展后的逆 */
    ufeq_status_t st = ufeq_status_ok;
    Ushort i = 0;
    Ushort j = 0;
    Ushort use_dim = 0; /* 实际参与 Cholesky 的维数 */

    if (in == NULL || out == NULL || dim == 0) {
        return ufeq_fail("matrix", "invalid_arg");
    }
    use_dim = dim;
    if (kernel_dim > dim) {
        use_dim = kernel_dim; /* 扩展到 kernel_dim 维运算 */
    }
    if (use_dim > ufeq_max_rx_ant) {
        return ufeq_fail("matrix", "out_of_range");
    }

    /* 构造 padded：有效区拷贝 in，扩展区对角置 1 */
    for (i = 0; i < use_dim; ++i) {
        for (j = 0; j < use_dim; ++j) {
            if (i < dim && j < dim) {
                padded[i * use_dim + j] = in[i * dim + j];
            } else if (i == j) {
                padded[i * use_dim + j] = ufeq_cf(1.0, 0.0); /* 扩展对角单位阵 */
            }
        }
    }

    /* 首次 Cholesky 求逆 */
    st = ufeq_cholesky_inverse(padded, padded_inv, use_dim, diag_load);
    if (st != ufeq_status_ok) {
        /* 失败则加大对角加载重试一次 */
        st = ufeq_cholesky_inverse(padded, padded_inv, use_dim, diag_load + 1e-3);
        if (st != ufeq_status_ok) {
            return st;
        }
    }

    /* 从 padded_inv 截取 dim×dim 有效部分到 out */
    for (i = 0; i < dim; ++i) {
        for (j = 0; j < dim; ++j) {
            out[i * dim + j] = padded_inv[i * use_dim + j];
        }
    }
    return ufeq_status_ok;
}
