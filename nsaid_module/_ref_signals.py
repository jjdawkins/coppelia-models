import numpy as np
from sympy import symbols, sin, lambdify, Array, diff, atan2


def init_ref_signals(self):
    # create the reference functions (use subs to evaluate)
    # t = symbols("t")
    # ref_v = Array(
    #     [
    #         2.7 + 0.2 * sin(1.41 * t + 2) + 0.5 * sin(0.5 * t),
    #         0.3 * sin(0.05 * t + 1) + 0.1 * sin(1.51 * t) + 0.15 * sin(0.31 * t),
    #         0,
    #     ]
    # )
    # self.ref_v = lambdify(t, ref_v, "numpy")
    # self.ref_v_dot = lambdify(t, diff(ref_v, t), "numpy")

    # # make CONSTANT reference values
    t =symbols("t")
    self.ref_v = lambdify(t, Array([0.8, 1.3, 0]), "numpy")
    self.ref_v_dot = lambdify(t, Array([0, 0, 0]), "numpy")

    # make figure 8 reference values
    # t = symbols("t")
    # speed = 0.4
    # x = 1.2 * sin(speed * 2 * t)
    # y = 4 * sin(speed * t)
    # dot_x = diff(x, t)
    # dot_y = diff(y, t)

    # theta = atan2(dot_y, dot_x)
    # theta_dot = diff(theta, t)

    # ref_v = Array([2.3 + 0.1 * sin(0.3 * t), theta_dot, 0])
    # ref_v = Array([2.0, theta_dot, 0])

    #self.ref_v = lambdify(t, ref_v, "numpy")
    #self.ref_v_dot = lambdify(t, diff(ref_v, t), "numpy")


def update_t(self, setInit=False):
    sec, nsec = self.get_clock().now().seconds_nanoseconds()
    if setInit:
        # first run, set the initial time
        self.t_init = sec + nsec * 1e-9
        self.t = self.t_init

    now = (sec + nsec * 1e-9) - self.t_init
    # update the time step
    self.t = now


def get_z_d(self, t):
    # will return the DESIRED z_dot_d and z_ddot_d for a given t as a numpy array
    ref_v = self.ref_v(t)
    ref_v_dot = self.ref_v_dot(t)

    # convert to numpy array
    ref_v = np.array(ref_v).astype(np.float64)
    ref_v_dot = np.array(ref_v_dot).astype(np.float64)


def update_z_d(self):
    # update the reference velocity and acceleration values
    self.z_dot_d = self.ref_v(self.t)
    self.z_ddot_d = self.ref_v_dot(self.t)
