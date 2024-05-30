import numpy as np


def eval_W_z(self):
    # Evaluate the 2x7 W_z matrix and return as a numpy array
    l = self.l
    x_ddot_d = self.z_ddot_d[0]
    psi_ddot_d = self.z_ddot_d[1]
    y_ddot_d = self.z_ddot_d[2]
    x_dot = self.z_dot[0]
    psi_dot = self.z_dot[1]
    y_dot = self.z_dot[2]
    m_hat = self.theta_h[0]
    j_z_hat = self.theta_h[1]
    k_hat = self.theta_h[2]
    c_rr_hat = self.theta_h[3]
    c_af_hat = self.theta_h[4]
    c_s_hat = self.theta_h[5]
    c_d_hat = self.theta_h[6]

    # init the W_z matrix
    W_z = np.zeros((2, 7))

    # fill in the values (Copied from MATLAB)
    W_z[0][0] = x_ddot_d-psi_dot*y_dot
    W_z[0][2] = -(c_rr_hat*x_dot)/k_hat - \
        (m_hat*(x_ddot_d-psi_dot*y_dot))/k_hat
    W_z[0][3] = x_dot
    W_z[1][1] = psi_ddot_d
    W_z[1][4] = -(c_d_hat*l*y_dot+j_z_hat*psi_ddot_d*x_dot +
                  c_s_hat*(l*l)*psi_dot)/(c_af_hat*x_dot)
    W_z[1][5] = ((l*l)*psi_dot)/x_dot
    W_z[1][6] = (l*y_dot)/x_dot

    self.W_z = W_z


def eval_control(self):
    # calculate the control inputs and return as a numpy array
    k1 = self.k1
    k2 = self.k2
    x_dot = self.z_dot[0]
    psi_dot = self.z_dot[1]
    y_dot = self.z_dot[2]
    x_dot_d = self.z_dot_d[0]
    psi_dot_d = self.z_dot_d[1]
    x_ddot_d = self.z_ddot_d[0]
    psi_ddot_d = self.z_ddot_d[1]
    m = self.theta_h[0]
    j_z = self.theta_h[1]
    k = self.theta_h[2]
    c_rr = self.theta_h[3]
    c_af = self.theta_h[4]
    c_s = self.theta_h[5]
    c_d = self.theta_h[6]
    l = self.l

    C = np.zeros(2)
    C[0] = -k1*(x_dot-x_dot_d)+(c_rr*x_dot+m*x_ddot_d-m*psi_dot*y_dot)/k
    C[1] = -k2*(psi_dot-psi_dot_d)+(j_z*psi_ddot_d +
                                    (l*(c_d*y_dot+c_s*l*psi_dot))/x_dot)/(c_af*l)

    self.C = C
