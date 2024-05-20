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
    from ._plot_vel import plot_vel, update_histories
    from ._ref_signals import update_t, get_z_d, update_z_d
    from ._run_loop import run_loop
    from ._sub_pub import odom_callback, send_cmd_vel

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
        ref_v = Array([2.7 + 0.2*sin(1.41*t + 2) + 0.5*sin(0.5 * t), 0.3 * sin(0.05 *
                      t + 1) + 0.1 * sin(1.51*t) + 0.15 * sin(0.31*t), 0])
        self.ref_v = lambdify(t, ref_v, 'numpy')
        self.ref_v_dot = lambdify(t, diff(ref_v, t), 'numpy')

        # get starting values
        self.update_z_d()

        # define the parameters
        self.l = 0.170
        #                          m       Jz   k c_rr c_af c_s  c_d
        self.theta_0 = np.array([4.378, 0.0715, 3,  3,  12,  20, 5])

        # make column vector of parameters
        self.theta_h = np.copy(self.theta_0)

        # make our controller gains
        self.k1 = 1.5  # throttle gain
        self.k2 = 0.03  # steering

        # make our adaptive gains  m      J_z  k    c_rr c_af c_s  c_d
        self.gamma = 1 * np.diag([5e-2, 5e-2, 1e-3, 1e-3, 10,   10,  10])

        # init the z_dot history vector
        self.z_dot_hist = np.zeros((3, 1))
        self.z_dot_d_hist = np.zeros((3, 1))
        self.t_hist = np.zeros((1, 1))

        # make axes
        self.fig, self.ax = plt.subplots(2, 1)
        self.ax[0].set_title("Forward Velocity")
        self.ax[0].set_xlabel("Time (s)")
        self.ax[0].set_ylabel("Velocity (m/s)")
        self.ax[1].set_title("Yaw Rate")
        self.ax[1].set_xlabel("Time (s)")
        self.ax[1].set_ylabel("Yaw Rate (rad/s)")

        self.lines = [Line2D([0], [0], color='blue', lw=2),
                      Line2D([0], [0], color='orange', lw=2, linestyle='--'),
                      Line2D([0], [0], color='blue', lw=2),
                      Line2D([0], [0], color='orange', lw=2, linestyle='--')]

        self.ax[0].add_line(self.lines[0])
        self.ax[0].add_line(self.lines[1])
        self.ax[1].add_line(self.lines[2])
        self.ax[1].add_line(self.lines[3])

        self.ax[0].legend(
            self.lines, ['Measured', 'Desired'], loc='upper left')
        self.ax[1].legend(
            self.lines, ['Measured', 'Desired'], loc='upper left')
        plt.draw()

        # start moving!
        self.send_cmd_vel(2.0, 0.0)
        time.sleep(1)
