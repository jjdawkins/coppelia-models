import sympy as sp
import os


# Build the System with sympy symbols
x_dot, y_dot, psi_dot = sp.symbols('x_dot y_dot psi_dot', real=True)
x_dot_d, y_dot_d, psi_dot_d = sp.symbols('x_dot_d y_dot_d psi_dot_d', real=True)
x_ddot, y_ddot, psi_ddot = sp.symbols('x_ddot y_ddot psi_ddot', real=True)
x_ddot_d, y_ddot_d, psi_ddot_d = sp.symbols('x_ddot_d y_ddot_d psi_ddot_d', real=True)
m, j_z, k, c_rr, c_af, c_s, c_d = sp.symbols('m j_z k c_rr c_af c_s c_d', real=True)
m_hat, j_z_hat, k_hat, c_rr_hat, c_af_hat, c_s_hat, c_d_hat = sp.symbols('m_hat j_z_hat k_hat c_rr_hat c_af_hat c_s_hat c_d_hat', real=True)
i_in, delta_in = sp.symbols('i_in delta_in', real=True)
a, b, c, d, e = sp.symbols('a b c d e', real=True)
k1, k2 = sp.symbols('k1 k2', real=True)
l = sp.symbols('l', real=True)

l = 0.16

# parameter vectors
theta = sp.Matrix([m, j_z, k, c_rr, c_af, c_s, c_d])
n = len(theta)
theta_hat = sp.Matrix([m_hat, j_z_hat, k_hat, c_rr_hat, c_af_hat, c_s_hat, c_d_hat])

# velocities
z_dot = sp.Matrix([x_dot, psi_dot, y_dot])
z_ddot = sp.Matrix([x_ddot, psi_ddot, y_ddot])
z_ddot_d = sp.Matrix([x_ddot_d, psi_ddot_d, y_ddot_d])
z_dot_d = sp.Matrix([x_dot_d, psi_dot_d, y_dot_d])

# inputs
u = sp.Matrix([i_in, delta_in])

# Define system dynamics D
D = sp.Matrix([
    k/m * i_in - c_rr/m * x_dot + y_dot*psi_dot,
    -(c_d  * l)/(j_z * x_dot) * y_dot - (c_s * l**2)/(j_z*x_dot) * psi_dot + (c_af* l)/(j_z) * delta_in,
    -(c_s * y_dot)/(m*x_dot) - (l*c_d)/(m*x_dot) *(psi_dot) + c_af/m * delta_in - x_dot *psi_dot
])

# split up to F and G st m*z_ddot = F(s,theta) + G(u,theta) 
M = sp.diag(m, j_z, m)
# we know what G should be (control portion)
G = sp.Matrix([k * i_in, c_af * l * delta_in, c_af * delta_in])
F = sp.simplify(M*D-G) 

# Now break up G into B and A:  G = B A(theta) * u
A = sp.diag(k, c_af)
B = sp.Matrix([[1, 0], [0, l], [0, 1]])

# check to make sure G = BAu
G_check = sp.simplify(G-B*A*u)

# Define the regressors as labeled by llw
W_m = sp.simplify(sp.diff(M*z_ddot, theta))
W_f = sp.simplify(sp.diff(F, theta))
W_g = sp.simplify(sp.diff(G, theta))