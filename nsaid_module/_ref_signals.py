import numpy as np


def update_t(self):
    sec, nsec = self.get_clock().now().seconds_nanoseconds()
    now = (sec + nsec * 1e-9) - self.t_init

    # update the time step
    self.dt = now - self.t
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
