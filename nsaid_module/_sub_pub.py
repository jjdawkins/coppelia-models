import numpy as np
from geometry_msgs.msg import Twist


def odom_callback(self, msg):
    # get the velocities from the subscriber
    x_dot = msg.twist.twist.linear.x
    psi_dot = msg.twist.twist.angular.z
    y_dot = msg.twist.twist.linear.y
    self.t_odom = self.t
    # update the z_dot vector
    self.z_dot = np.array([x_dot, psi_dot, y_dot])


def send_cmd_vel(self, omega, delta):
    twist = Twist()
    twist.linear.x = omega
    twist.angular.z = delta
    self.vel_pub.publish(twist)
