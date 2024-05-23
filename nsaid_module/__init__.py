from rclpy.node import Node
import numpy as np
from numpy.random import randn
import time
from sympy import Matrix, symbols, sin, lambdify, Array, diff
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt


class nsaidEstimation(Node):
    # import methods from other files
    from ._eval_mat import eval_W_z, eval_control
    from ._plot_vel import (
        plot_vel,
        update_histories,
        init_vel_plot,
        init_error_plot,
        plot_error,
    )
    from ._plot_params import init_param_plot, plot_params
    from ._ref_signals import update_t, get_z_d, update_z_d, init_ref_signals
    from ._run_loop import run_loop
    from ._sub_pub import odom_callback, send_cmd_vel
    from ._set_gains import init_gains

    def __init__(self):
        super().__init__("nsaid_estimation")
        print("nsaid_estimation node started")

        # call init functions
        self.init_vel_plot()
        self.init_error_plot()
        self.init_gains()
        self.init_param_plot()
        self.update_t(setInit=True)
        self.init_ref_signals()

        self.dot_n = 0
        self.twirl = ["|", "/", "-", "\\"]

        # publish to cmd vel topic
        self.vel_topic = "/rover/cmd_vel"
        self.vel_pub = self.create_publisher(Twist, self.vel_topic, 10)

        # subscribe to the odometry topic
        self.t_odom = self.t
        self.odom_sub = self.create_subscription(
            Odometry, "/rover/mocap/odom", self.odom_callback, 10
        )

        # create timer that executes the run loop (NOT USED IF RUN ONCE)
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.run_loop)

        # get starting values
        self.z_dot = np.zeros(3)
        self.update_z_d()

        # init the z_dot history vector
        self.z_dot_hist = np.zeros((3, 1))
        self.z_dot_d_hist = np.zeros((3, 1))
        self.t_hist = np.zeros((1, 1))

        # start moving!
        self.send_cmd_vel(2.0, 0.0)
        time.sleep(1)
