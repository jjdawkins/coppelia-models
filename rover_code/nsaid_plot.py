from rclpy.node import Node
import rclpy
import matplotlib.pyplot as plt
import time
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np
from time import sleep


class nsaidPlot(Node):
    from nsaid_plot_functions.plot_params import init_param_plot, plot_params
    from nsaid_plot_functions.plot_vel import (
        init_vel_plot,
        init_error_plot,
        plot_vel,
        plot_error,
    )
    from nsaid_plot_functions.plot_inputs import init_input_plot, plot_input

    def __init__(self):
        super().__init__("nsaid_plot")
        print("nsaid_plot node started")

        # subscribe to the est_param topic
        self.est_param_topic = "/rover/est_param"
        self.est_param_sub = self.create_subscription(
            Float32MultiArray, self.est_param_topic, self.est_param_callback, 10
        )

        self.t_hist = np.array([])
        self.t_z_dot_hist = np.array([])
        self.t_z_dot_d_hist = np.array([])
        self.t_u_hist = np.array([])

        self.init_param_plot()
        self.init_vel_plot()
        # self.init_error_plot()
        self.init_input_plot()

        # subscribe to the ref_vel topic
        self.ref_vel_topic = "/rover/ref_vel"
        self.ref_vel_sub = self.create_subscription(
            Float32MultiArray, self.ref_vel_topic, self.ref_vel_callback, 10
        )

        # subscribe to the act_vel topic
        self.act_vel_topic = "/rover/act_vel"
        self.act_vel_sub = self.create_subscription(
            Float32MultiArray, self.act_vel_topic, self.act_vel_callback, 10
        )

        # subscribe to cmd_vel topic
        self.cmd_vel_topic = "/rover/cmd_vel"
        self.cmd_vel_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10
        )

        # create timer that executes the run loop
        self.dt = 0.4  # SPEED UP OR SLOW DOWN THE PLOT
        self.timer = self.create_timer(self.dt, self.run_loop)
        self.ii = 0

        self.u = np.array([[0], [0]])  # 2x1 array

    def cmd_vel_callback(self, msg):
        # just save the input, add it later with another callback
        self.u = np.array([[msg.linear.x], [msg.angular.z]])

    def ref_vel_callback(self, msg):
        n = len(msg.data) - 1
        self.t_z_dot_d_hist = np.append(self.t_z_dot_d_hist, msg.data[0])
        z_dot_d = np.reshape(msg.data[1:], (n, -1))
        if hasattr(self, "z_dot_d_hist") is False:
            self.z_dot_d_hist = np.array(z_dot_d)
        else:
            self.z_dot_d_hist = np.append(self.z_dot_d_hist, z_dot_d, axis=1)

    def act_vel_callback(self, msg):
        n = len(msg.data) - 1
        self.t_z_dot_hist = np.append(self.t_z_dot_hist, msg.data[0])
        self.z_dot_t = msg.data[0]
        z_dot = np.reshape(msg.data[1:], (n, -1))
        if hasattr(self, "z_dot_hist") is False:
            self.z_dot_hist = np.array(z_dot)
        else:
            self.z_dot_hist = np.append(self.z_dot_hist, z_dot, axis=1)

    def est_param_callback(self, msg):
        n = len(msg.data) - 1
        # split to time and parameters
        self.t_hist = np.append(self.t_hist, msg.data[0])
        # reshape theta h to 7x1
        theta_h = np.reshape(msg.data[1:], (n, -1))
        if self.t_hist.shape[0] == 1:
            self.theta_h_hist = np.array(theta_h)
            self.u_hist = np.array(self.u)
        else:
            self.theta_h_hist = np.append(self.theta_h_hist, theta_h, axis=1)
            self.u_hist = np.append(self.u_hist, self.u, axis=1)

    def run_loop(self):
        pass


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    nsaid_plot = nsaidPlot()

    plt.ion()

    try:
        while rclpy.ok():
            # this updates the plot data callbacks
            rclpy.spin_once(nsaid_plot, timeout_sec=0.1)

            # update the plots
            if nsaid_plot.t_hist.shape[0] > 0:
                if nsaid_plot.ii == 0:
                    nsaid_plot.plot_params()
                elif nsaid_plot.ii == 1:
                    nsaid_plot.plot_vel()
                elif nsaid_plot.ii == 2:
                    # nsaid_plot.plot_error()
                    nsaid_plot.plot_input()
                    pass
                nsaid_plot.ii = (nsaid_plot.ii + 1) % 3
                plt.pause(1e-4)

    except KeyboardInterrupt:
        print("Keyboard Interrupt")

    finally:
        # Clean up and destroy the node
        nsaidPlot.destroy_node(nsaid_plot)

        # Shut down the ROS 2 Python client library
        rclpy.shutdown()


if __name__ == "__main__":
    main()
