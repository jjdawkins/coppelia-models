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
    from ._ref_signals import update_t, get_z_d, update_z_d, init_ref_signals
    from ._run_loop import run_loop
    from ._sub_pub import odom_callback, send_cmd_vel
    from ._set_gains import init_gains
    from ._eval_mat import create_C_Wz

    def __init__(self):
        super().__init__("nsaid_estimation")
        print("nsaid_estimation node started")

        # call init functions

        self.init_gains()
        self.update_t(setInit=True)
        self.init_ref_signals()
        self.C_func, self.W_z_func = self.create_C_Wz()

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

        # start moving!
        self.send_cmd_vel(2.0, 0.0)
        time.sleep(1)
