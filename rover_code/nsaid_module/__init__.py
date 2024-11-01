from rclpy.node import Node
import numpy as np
from numpy.random import randn
import time
from sympy import Matrix, symbols, sin, lambdify, Array, diff
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu


class nsaidEstimation(Node):
    # import methods from other files
    from ._ref_signals import update_t, get_z_d, update_z_d, init_ref_signals
    from ._run_loop import run_loop
    from ._sub_pub import odom_callback, send_cmd_vel, imu_callback
    from ._set_gains import init_gains
    from ._eval_mat import create_C_Wz

    def __init__(self):
        super().__init__("nsaid_estimation")
        print("nsaid_estimation node started")

        self.last_psi_dot = 0.0

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

        # subscript to the imu
        self.imu_sub = self.create_subscription(
            Imu, "/rover/imu", self.imu_callback, 10
        )

        # publish the est_param topic (shold be list of floats)
        self.est_param_topic = "/rover/est_param"
        self.est_param_pub = self.create_publisher(
            Float32MultiArray, self.est_param_topic, 10
        )

        # publish the reference vel
        self.ref_vel_topic = "/rover/ref_vel"
        self.ref_vel_pub = self.create_publisher(
            Float32MultiArray, self.ref_vel_topic, 10
        )

        # publish the actual vel
        self.act_vel_topic = "/rover/act_vel"
        self.act_vel_pub = self.create_publisher(
            Float32MultiArray, self.act_vel_topic, 10
        )

        # create timer that executes the run loop
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.run_loop)

        # get starting values
        self.z_dot = np.zeros(3)
        self.update_z_d()

        # start moving!
        self.send_cmd_vel(2.0, 0.0)
        time.sleep(1)
