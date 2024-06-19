from rclpy.node import Node
import rclpy
import matplotlib.pyplot as plt
import time
from std_msgs.msg import Float32MultiArray
import numpy as np


class nsaidPlot(Node):
    from nsaid_plot_functions.plot_params import init_param_plot, plot_params

    def __init__(self):
        super().__init__("nsaid_plot")
        print("nsaid_plot node started")

        # subscribe to the est_param topic
        self.est_param_topic = "/rover/est_param"
        self.est_param_sub = self.create_subscription(
            Float32MultiArray, self.est_param_topic, self.est_param_callback, 10
        )

        self.t_hist = np.array([])

        self.init_param_plot()

        # create timer that executes the run loop
        self.dt = 0.5  # SPEED UP OR SLOW DOWN THE PLOT
        self.timer = self.create_timer(self.dt, self.run_loop)

    def est_param_callback(self, msg):
        n = len(msg.data) - 1
        # split to time and parameters
        self.t_hist = np.append(self.t_hist, msg.data[0])
        # reshape theta h to 7x1
        theta_h = np.reshape(msg.data[1:], (n, -1))
        if self.t_hist.shape[0] == 1:
            self.theta_h_hist = np.array(theta_h)
        else:
            self.theta_h_hist = np.append(self.theta_h_hist, theta_h, axis=1)

    def run_loop(self):
        if self.t_hist.shape[0] > 0:
            self.plot_params()
            plt.pause(1e-4)


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    nsaid_plot = nsaidPlot()

    plt.ion()

    while rclpy.ok():
        # Process any pending events and callbacks
        rclpy.spin_once(nsaid_plot)

    # Clean up and destroy the node
    nsaidPlot.destroy_node()

    # Shut down the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == "__main__":
    main()
