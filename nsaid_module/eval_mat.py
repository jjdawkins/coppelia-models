# uses sympy to create W_z function and C function
import sympy as sp


def create_Wz_C(self):
    # IMPORTANT DEFINE L
    l = self.l  # 0.17 for coppeliasim rover

    # make theta variables
    m, j_z, k, c_rr, c_af, c_s, c_d = sp.symbols(
        r"m j_z k c_rr c_af c_s c_d", real=True
    )

    # define measured state values
    x_dot, y_dot, psi_dot = sp.symbols("x_dot y_dot psi_dot", real=True)
    x_ddot, y_ddot, psi_ddot = sp.symbols("x_ddot y_ddot psi_ddot", real=True)
    # define desired values
    x_dot_d, y_dot_d, psi_dot_d = sp.symbols("x_dot_d y_dot_d psi_dot_d", real=True)
    x_ddot_d, y_ddot_d, psi_ddot_d = sp.symbols(
        "x_ddot_d y_ddot_d psi_ddot_d", real=True
    )

    k1, k2 = sp.symbols("k_1 k_2", real=True)
    k_vec = sp.Matrix([k1, k2])
    k_diag = sp.diag(k1, k2)

    # inputs
    i_in, delta_in = sp.symbols("i_in delta_in")
    u = sp.Matrix([i_in, delta_in])

    # Make state vectors
    z_dot = sp.Matrix([x_dot, psi_dot, y_dot])
    z_ddot = sp.Matrix([x_ddot, psi_ddot, y_ddot])
    z_dot_d = sp.Matrix([x_dot_d, psi_dot_d, y_dot_d])
    z_ddot_d = sp.Matrix([x_ddot_d, psi_ddot_d, y_ddot_d])

    # Make the theta vector
    theta = sp.Matrix([m, j_z, k, c_rr, c_af, c_s, c_d])

    # Mass matrix
    M = sp.Matrix([[m, 0, 0], [0, j_z, 0], [0, 0, m]])

    # Dynamics matrix z_ddot = D(z, u, theta)
    D = sp.Matrix(
        [
            [k / m * i_in - c_rr / m * x_dot + y_dot * psi_dot],
            [
                -(c_d * l) / (j_z * x_dot) * y_dot
                - (c_s * l**2) / (j_z * x_dot) * psi_dot
                + (c_af * l) / (j_z) * delta_in
            ],
            [
                -(c_s * y_dot) / (m * x_dot)
                - (l * c_d) / (m * x_dot) * (psi_dot)
                + c_af / m * delta_in
                - x_dot * psi_dot
            ],
        ]
    )

    # split up to F and G st m*z_ddot = F(s,theta) + G(u,theta)
    D_m = M @ D  # D_m is M * z_ddot = D_m
    sp.pprint(D_m)

    G_u = D_m.jacobian(u)
    G_u.simplify()
    print(f"G_u:\n")
    sp.pprint(G_u)

    G = G_u @ u
    print(f"G:\n")
    sp.pprint(G)

    F = D_m - G
    F.simplify()

    # make sure that D_m = F + Gu
    check1 = D_m - (F + G)
    assert check1.simplify() == None

    # get B and A st G = B * A * u
    A = sp.diag(k, c_af)
    print(f"A:\n")
    sp.pprint(A)
    W_a_vec = (A @ sp.ones(2, 1)).jacobian(theta)
    print(f"W_a_vec:\n")
    sp.pprint(W_a_vec)
    B = G_u @ sp.Inverse(A)
    B.simplify()
    B_bar = B[:2, :]
    sp.pprint(B_bar)

    # check that G = B * A * u
    check2 = G - B @ A @ u
    assert check2.simplify() == None

    W_m = (M @ z_ddot_d).jacobian(theta)
    print(f"W_m:\n")
    sp.pprint(W_m)
    W_m.simplify()
    W_f = F.jacobian(theta)
    W_f.simplify()
    W_g = G.jacobian(theta)
    W_g.simplify()

    # define controller regressor
    W_m_bar = W_m[:2, :]
    W_f_bar = W_f[:2, :]
    W_c = W_m_bar - W_f_bar
    sp.pprint(W_c)

    # define the controller
    delta_z_dot = z_dot - z_dot_d
    delta_z_dot_bar = delta_z_dot[:2, :]
    C = sp.Inverse(A) @ sp.Inverse(B_bar) @ W_c @ theta - k_diag @ delta_z_dot_bar
    C.simplify()
    sp.pprint(C)

    # check the controlled dynamics by substituting C into the dynamics
    print(f"Controlled Dynamics:\n")
    controlled_dynamics = D_m.subs(zip(u,C)) - M @ z_ddot_d
    controlled_dynamics.simplify()
    sp.pprint(controlled_dynamics)

    C_func = sp.lambdify([z_ddot_d, z_dot_d, z_dot, theta, k_vec], C, "numpy")

    # MAKE W_z FUNCTION ########################################
    p = theta.shape[0] # should be 7
    delta_theta = sp.Matrix([sp.symbols(f"Delta-theta{i}") for i in range(p)])
    theta_hat = sp.Matrix([sp.symbols(f"theta-hat{i}") for i in range(p)])

    W_delta = A.subs(zip(theta, delta_theta)) @ sp.Inverse(A.subs(zip(theta, theta_hat))) @ W_c @ theta_hat

    W_delta = W_delta.jacobian(delta_theta)

    W_delta.simplify()
    print(f"W_delta:\n")
    sp.pprint(W_delta)

    W_z = W_c - W_delta
    W_z.simplify()
    W_z = W_z.subs(zip(theta_hat, theta))
    print(f"W_z:\n") 
    sp.pprint(W_z)

    W_z_func = sp.lambdify([z_ddot_d, z_dot, theta], W_z, "numpy")

    return C_func, W_z_func


if __name__ == "__main__":

    class emptyClass:
        pass

    # pretty print the function
    sp.init_printing()

    self = emptyClass()
    self.l = 0.17
    create_Wz_C(self)
