# uses sympy to create W_z function and C function
import sympy as sp


def create_Wz_C(self):
    # IMPORTANT DEFINE L
    l = self.l  # 0.17 for coppeliasim rover

    # make theta variables
    m, j_z, k, c_rr, c_af, c_s, c_d = sp.symbols("m j_z k c_rr c_af c_s c_d", real=True)
    # define measured state values
    x_dot, y_dot, psi_dot = sp.symbols("x_dot y_dot psi_dot", real=True)
    x_ddot, y_ddot, psi_ddot = sp.symbols("x_ddot y_ddot psi_ddot", real=True)
    # define desired values
    x_dot_d, y_dot_d, psi_dot_d = sp.symbols("x_dot_d y_dot_d psi_dot_d", real=True)
    x_ddot_d, y_ddot_d, psi_ddot_d = sp.symbols(
        "x_ddot_d y_ddot_d psi_ddot_d", real=True
    )

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


if __name__ == "__main__":

    class emptyClass:
        pass

    # pretty print the function
    sp.init_printing()

    self = emptyClass()
    self.l = 0.17
    create_Wz_C(self)
