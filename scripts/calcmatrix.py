# 计算 src/pnc_planner/src/quintic_polynomial.cpp文件中的c3_, c4_, c5_

import sympy as sp

T = sp.symbols("T")
A, B, C = sp.symbols("A B C")

M = sp.Matrix(
    [
        [T**3, T**4, T**5],
        [3 * T**2, 4 * T**3, 5 * T**4],
        [6 * T, 12 * T**2, 20 * T**3],
    ]
)

M_inv = M.inv()

Y = sp.Matrix([A, B, C])

result = M_inv * Y

print("c3_ = " + sp.ccode(sp.simplify(result[0])) + ";")
print("c4_ = " + sp.ccode(sp.simplify(result[1])) + ";")
print("c5_ = " + sp.ccode(sp.simplify(result[2])) + ";")
