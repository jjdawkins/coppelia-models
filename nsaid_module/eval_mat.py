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
    x_dot, y_dot, psi_dot = sp.symbols("x_dot psi_dot y_dot", real=True)
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
    z = sp.Matrix([x_dot, psi_dot, y_dot])
    z_dot = sp.Matrix([x_ddot, psi_ddot, y_ddot])
    z_d = sp.Matrix([x_dot_d, psi_dot_d, y_dot_d])
    z_dot_d = sp.Matrix([x_ddot_d, psi_ddot_d, y_ddot_d])

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
    sp.pprint(G_u)

    G = G_u @ u
    sp.pprint(G)

    F = D_m - G
    F.simplify()

    # make sure that D_m = F + Gu
    check1 = D_m - (F + G)
    assert check1.simplify() == None

    # get B and A st G = B * A * u
    B = sp.Matrix([[1, 0], [0, l], [0, 1]])
    B_bar = B[:2, :]
    A = sp.diag(k, i_in)

    # check that G = B * A * u
    check2 = G - B @ A @ u
    assert check2.simplify() == None

    W_m = (M @ z_dot).jacobian(theta)
    W_f = F.jacobian(theta)
    W_g = G.jacobian(theta)

    # define controller regressor
    W_m_bar = W_m[:2, :]
    W_f_bar = W_f[:2, :]
    W_c = W_m_bar - W_f_bar

    # define the controller
    delta_z_dot = z_dot - z_dot_d
    delta_z_dot_bar = delta_z_dot[:2, :]
    C = sp.Inverse(A) @ B_bar @ W_c @ theta - k_diag @ delta_z_dot_bar
    C.simplify()
    sp.pprint(C)

    C_func = sp.lambdify([z_dot_d, z_d, z_dot, theta, k_vec], C, "numpy")


if __name__ == "__main__":

    class emptyClass:
        pass

    # pretty print the function
    sp.init_printing()

    self = emptyClass()
    self.l = 0.17
    create_Wz_C(self)
