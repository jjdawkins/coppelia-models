import numpy as np


def init_gains(self):
    # define the parameters
    self.l = 0.170
    #                         m       Jz    k   c_rr c   _af c_s  c_d
    self.theta_0 = np.array([4.378, 0.0715, 3.0, 3.0, 12.0, 20.0, 5.0])

    # make column vector of ESTIMATED (Hat) parameters
    self.theta_h = np.copy(self.theta_0)

    # make our controller gains
    self.k1 = 1.5  # throttle gain
    self.k2 = 0.03  # steering

    # make our adaptive gains  m    J_z  k     c_rr c_af c_s  c_d
    self.gamma = 1 * np.diag([5e-2, 5e-2, 1e-3, 1e-3, 10, 10, 10])
