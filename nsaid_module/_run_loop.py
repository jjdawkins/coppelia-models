import numpy as np
from std_msgs.msg import Float32MultiArray


def run_loop(self):
    # default speed
    default_speed = 0.6
    # max speed = 1.5
    max_speed = 1.0
    min_speed = 0.6

    # update the time
    self.update_t()

    # make sure that messages are being received
    if self.t - self.t_odom > 0.2:

        self.dot_n = (self.dot_n + 1) % len(self.twirl)
        print(f"No messages received! {self.twirl[self.dot_n] + ' '*8} ", end="\r")
        self.send_cmd_vel(1.0, 0.0)
        return

    # make sure speed is not zero
    if self.z_dot[0] < 0.1:
        print(f"Speed too low! {self.z_dot[0]:.2f} m/s", end="\r")
        self.send_cmd_vel(default_speed, 0.0)
        return

    # print(f"t: {self.t:.2f}")
    # get delta_z_dot (this is a 1x3)
    delta_z_dot = self.z_dot - self.z_dot_d

    # get new reference velocity values
    self.update_z_d()

    # Evaluate W_z (this is a 2x7 matrix)
    # W_z_func = sp.lambdify([self, z_ddot_d, z_dot, theta], W_z, "numpy", dummify=True)
    # returns a 2x7 numpy array
    self.W_z = self.W_z_func(self.z_ddot_d, self.z_dot, self.theta_h)

    # Calculate the parameter update increment
    W_z_T = np.transpose(self.W_z)  # 7x2
    delta_z_dot2 = np.reshape(delta_z_dot[0:2], (2, 1))  # 2x1
    delta_theta_h = -self.gamma @ W_z_T @ delta_z_dot2
    # force a 1x7 shape
    delta_theta_h = np.reshape(delta_theta_h, 7)

    # apply the parameter updated
    self.theta_h = self.theta_h + delta_theta_h * self.dt

    # update the control inputs
    # [self, z_ddot_d, z_dot_d, z_dot, theta, k_vec], C, "numpy", dummify=True
    # returns a 2x1 numpy array
    self.C = self.C_func(
        self.z_ddot_d, self.z_dot_d, self.z_dot, self.theta_h, self.k_vec
    )

    cmd_speed = self.C[0, 0]

    print(cmd_speed)

    if cmd_speed > max_speed:
        cmd_speed = max_speed
    elif cmd_speed < min_speed:
        cmd_speed = min_speed


    # send the control inputs # CHANGE TO CONTROL WHEEL SPEED ####################
    self.send_cmd_vel(cmd_speed, self.C[1, 0])

    # publish the estimated parameters
    msg = Float32MultiArray()
    # first add time to the message
    msg.data = [self.t, *self.theta_h]
    # then publish the message
    self.est_param_pub.publish(msg)

    # publish the reference signals
    msg = Float32MultiArray()
    # first add time to the message
    msg.data = [self.t, *self.z_dot_d]
    # then publish the message
    self.ref_vel_pub.publish(msg)

    # publish actual velocity
    msg = Float32MultiArray()
    # first add time to the message
    msg.data = [self.t, *self.z_dot]
    # then publish the message
    self.act_vel_pub.publish(msg)
