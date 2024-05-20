import numpy as np


def run_loop(self):
    # update the time
    self.update_t()

    # make sure that messages are being received
    if self.t - self.t_odom > 0.2:
        print("No messages received!", end="\r")
        self.send_cmd_vel(2.0, 0.0)
        return

    # make sure speed is not zero
    if self.z_dot[0] < 0.2:
        print("Speed too low!")
        self.send_cmd_vel(2.0, 0.0)
        return

    # print(f"t: {self.t:.2f}")
    # get delta_z_dot (this is a 1x3)
    delta_z_dot = self.z_dot - self.z_dot_d

    # Evaluate W_z (this is a 2x7 matrix)
    self.eval_W_z()

    # Calculate the parameter update increment
    W_z_T = np.transpose(self.W_z)  # 7x2
    delta_z_dot2 = np.reshape(delta_z_dot[0:2], (2, 1))  # 2x1
    delta_theta_h = -self.gamma @ W_z_T @ delta_z_dot2
    # force a 1x7 shape
    delta_theta_h = np.reshape(delta_theta_h, 7)

    # print each theata_h value with constant width
    # for i in range(7):
    #     print(f"{self.theta_h[i]:.4f}", end=" ")

    # apply the parameter updated
    self.theta_h = self.theta_h + delta_theta_h * self.dt

    # get new reference velocity values
    self.update_z_d()

    # update the histories
    self.update_histories()

    # update the control inputs
    self.eval_control()

    # send the control inputs
    self.send_cmd_vel(self.C[0], self.C[1])
