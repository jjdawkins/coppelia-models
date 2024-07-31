import numpy as np


def init_gains(self):
    # define the parameters
    self.l = 0.170
    #                         m       Jz    k   c_rr    c_af c_s  c_d
    # self.theta_0 = np.array([4.378, 0.0715, 3.0, 3.0, 12.0, 20.0, 5.0])

    #                         m       Jz    k   c_rr  c_af  c_s  c_d
    self.theta_0 = np.array([4.378, 0.0715, 0.1, 0.1, 20.0, 20.0, 13.0])
    # make column vector of ESTIMATED (Hat) parameters
    self.theta_h = np.copy(self.theta_0)

    self.p = self.theta_h.shape[0]

    # add to the parameter history (tranpose to make it a column vector)
    self.theta_h_hist = np.array(np.reshape(self.theta_h, (self.p, 1)))

    # make our controller gains
    self.k1 = 1.0  # throttle gain
    self.k2 = 0.08  # steering gain (0.05 worked)
    self.k_vec = np.array([self.k1, self.k2])

    # make our adaptive gains  m    J_z    k     c_rr c_af c_s  c_d
    self.gamma = 10 * np.diag([1e-2, 1e-2, 1e-3, 1e-3, 10, 10, 10])
