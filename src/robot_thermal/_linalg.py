"""The handful of small dense-matrix operations the model, observer and fit need. Standard library only.

WHY hand-rolled: the runtime core must import with nothing installed (tests/test_boundary.py), and every matrix
here is at most about 12×12. numpy is the oracle in tests/test_linalg.py.
"""

from __future__ import annotations

import math

Vec = list[float]
Mat = list[list[float]]


def zeros(n: int, m: int | None = None) -> Mat:
    return [[0.0] * (n if m is None else m) for _ in range(n)]


def eye(n: int) -> Mat:
    out = zeros(n)
    for i in range(n):
        out[i][i] = 1.0
    return out


def matmul(a: Mat, b: Mat) -> Mat:
    bt = list(zip(*b))
    return [[sum(x * y for x, y in zip(row, col)) for col in bt] for row in a]


def matvec(a: Mat, v: Vec) -> Vec:
    return [sum(x * y for x, y in zip(row, v)) for row in a]


def transpose(a: Mat) -> Mat:
    return [list(r) for r in zip(*a)]


def add(a: Mat, b: Mat, scale: float = 1.0) -> Mat:
    return [[x + scale * y for x, y in zip(ra, rb)] for ra, rb in zip(a, b)]


def scale(a: Mat, s: float) -> Mat:
    return [[x * s for x in row] for row in a]


def solve(a: Mat, b: Vec) -> Vec:
    """x with a·x = b, Gaussian elimination with partial pivoting."""
    n = len(a)
    m = [list(row) + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-300:
            raise ZeroDivisionError("singular matrix")
        m[col], m[piv] = m[piv], m[col]
        p = m[col][col]
        for r in range(col + 1, n):
            f = m[r][col] / p
            if f:
                mr, mc = m[r], m[col]
                for c in range(col, n + 1):
                    mr[c] -= f * mc[c]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][c] * x[c] for c in range(r + 1, n))) / m[r][r]
    return x


def inverse(a: Mat) -> Mat:
    n = len(a)
    cols = [solve(a, [1.0 if i == j else 0.0 for i in range(n)]) for j in range(n)]
    return transpose(cols)


def expm(a: Mat) -> Mat:
    """e^A by scaling and squaring a Taylor series; accurate to ~1e-12 for the small, stable matrices here."""
    n = len(a)
    norm = max(sum(abs(x) for x in row) for row in a)
    squarings = max(0, math.ceil(math.log2(norm / 0.5))) if norm > 0.5 else 0
    s = scale(a, 1.0 / (2**squarings))
    out, term = eye(n), eye(n)
    for k in range(1, 14):
        term = scale(matmul(term, s), 1.0 / k)
        out = add(out, term)
    for _ in range(squarings):
        out = matmul(out, out)
    return out
