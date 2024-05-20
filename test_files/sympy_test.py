from sympy import Matrix, Function, symbols, sin, cos, Derivative, Array, diff, lambdify
import numpy as np

t = symbols('t')
ref_v = ref_v = Array([1.5 + 0.5*sin(t), 0.3 * sin(0.11*t)])
ref_v_dot = diff(ref_v, t)
ref_v = lambdify(t, ref_v, 'numpy')
ref_v_dot = lambdify(t, ref_v_dot, 'numpy')

print(ref_v(0.0))
print(ref_v_dot(0.0))
