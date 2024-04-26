#!/bin/python3
import rclpy
import numpy as np
from numpy.random import randn
import quaternion
import math
import time
from rclpy.node import Node
import matplotlib.pyplot as plt

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, Pose, Vector3
from sensor_msgs.msg import Imu
from sympy import Matrix, symbols, sin, lambdify, Array, diff


class nsaidEstimation(Node):
    def __init__(self):
        super().__init__('nsaid_estimation')
        print("nsaid_estimation node started")

        # create time variables
        sec, nsec = self.get_clock().now().seconds_nanoseconds()
        self.t_init = sec + nsec * 1e-9
        self.t = self.t_init
        self.dt = 0.01

        # publish to cmd vel topic
        self.vel_topic = '/rover/cmd_vel'
        self.vel_pub = self.create_publisher(Twist, self.vel_topic, 10)

        # subscribe to the odometry topic
        self.odom_sub = self.create_subscription(
            Odometry, '/rover/mocap/odom', self.odom_callback, 10)
        self.z_dot = np.zeros(3)
        self.t_odom = -5

        # create timer that sends velocity commands
        self.dt = 0.01
        self.timer = self.create_timer(0.05, self.run_loop)

        # create the reference functions (use subs to evaluate)
        t = symbols('t')
        ref_v = Array([2.5 + 0.5*sin(1 * t), 0.5 + 1.5 * sin(0.3*t), 0])
        self.ref_v = lambdify(t, ref_v, 'numpy')
        self.ref_v_dot = lambdify(t, diff(ref_v, t), 'numpy')

        # get starting values
        self.update_z_d()

        # define the parameters
        self.l = 0.170
        #                          m       Jz   k c_rr c_af c_s  c_d
        self.theta_0 = np.array([4.378, 0.0715, 3,  3,  15, 20, -35])

        # make column vector of parameters
        self.theta_h = np.copy(self.theta_0)

        # make our controller gains
        self.k1 = 2
        self.k2 = 1
        # make our adaptive gains
        self.gamma = 10 * np.diag([1e-3, 1e-3, 1e-2, 1e-2, 1, 1, 1])

        # start moving!
        self.send_cmd_vel(2.0, 0.0)
        time.sleep(1)

    def get_z_d(self, t):
        # will return the z_dot_d and z_ddot_d for a given t as a numpy array
        ref_v = self.ref_v(t)
        ref_v_dot = self.ref_v_dot(t)

        # convert to numpy array
        ref_v = np.array(ref_v).astype(np.float64)
        ref_v_dot = np.array(ref_v_dot).astype(np.float64)

        return ref_v, ref_v_dot

    def send_cmd_vel(self, omega, delta):
        twist = Twist()
        twist.linear.x = omega
        twist.angular.z = delta
        self.vel_pub.publish(twist)

    def update_t(self):
        sec, nsec = self.get_clock().now().seconds_nanoseconds()
        now = (sec + nsec * 1e-9) - self.t_init

        # update the time step
        self.dt = now - self.t
        self.t = now

    def odom_callback(self, msg):
        # get the velocities from the subscriber
        x_dot = msg.twist.twist.linear.x
        psi_dot = msg.twist.twist.angular.z
        y_dot = msg.twist.twist.linear.y

        # update the z_dot vector
        self.z_dot = np.array([x_dot, psi_dot, y_dot])

        # update the time
        self. t_odom = self.t

    def update_z_d(self):
        # update the reference velocity and acceleration values
        self.z_dot_d = self.ref_v(self.t)
        self.z_ddot_d = self.ref_v_dot(self.t)

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

    def run_loop(self):
        # update the time
        self.update_t()

        # make sure that messages are being received
        if self.t - self.t_odom > 0.2:
            print("No messages received!")
            self.send_cmd_vel(2.0, 0.0)
            return

        # make sure speed is not zero
        if self.z_dot[0] < 0.5:
            print("Speed too low!")
            self.send_cmd_vel(2.0, 0.0)
            return

        print(f"t: {self.t:.2f}")
        # get delta_z_dot (this is a 1x3)
        delta_z_dot = self.z_dot - self.z_dot_d

        # Evaluate W_z (this is a 2x7 matrix)
        self.eval_W_z()

        # Calculate the update increment
        W_z_T = np.transpose(self.W_z)  # 7x2
        delta_z_dot2 = np.reshape(delta_z_dot[0:2], (2, 1))  # 2x1
        delta_theta_h = -self.gamma @ W_z_T @ delta_z_dot2
        # force a 1x7 shape
        delta_theta_h = np.reshape(delta_theta_h, 7)

        # print each theata_h value with constant width
        for i in range(7):
            print(f"{self.theta_h[i]:.4f}", end=" ")

        # apply the parameter updated
        self.theta_h = self.theta_h + delta_theta_h * self.dt

        # update the control inputs
        self.eval_control()

        # get new reference velocity values
        self.update_z_d()

        # send the control inputs
        self.send_cmd_vel(self.C[0], self.C[1])


### DONE WITH CLASS DEFINITION ###

def ros_spin(node):
    rclpy.spin(node)


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the parameterEstimation class
    nsaid_est = nsaidEstimation()

    # Run the main loop until the ROS 2 context is still valid
    rclpy.spin(nsaid_est)

    # Clean up and destroy the node
    nsaidEstimation.destroy_node()

    # Print a message indicating that the run has finished
    print("Run Finished")

    # Shut down the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
