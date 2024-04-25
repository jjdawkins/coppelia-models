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


class nsaidEstimation(Node):
    def __init__(self):
        super().__init__('nsaid_estimation')
        print("nsaid_estimation node started")
        # create time variables
        self.t_init = self.get_clock().now().nanoseconds * 1e-9

        # publish to cmd vel topic
        self.vel_topic = '/rover/cmd_vel'
        self.vel_pub = self.create_publisher(Twist, self.vel_topic, 10)

        # create timer that sends velocity commands
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.run_loop)

    def send_cmd_vel(self, u, w):
        twist = Twist()
        twist.linear.x = 2.0
        twist.angular.z = w
        self.vel_pub.publish(twist)

    def run_loop(self):
        self.send_cmd_vel(1.0, 0.1)


### DONE WITH CLASS DEFINITION ###

def ros_spin(node):
    rclpy.spin(node)


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the parameterEstimation class
    nsaid_est = nsaidEstimation()

    # Run the main loop until the ROS 2 context is still valid
    while (rclpy.ok):
        # Process any pending events and callbacks
        rclpy.spin_once(nsaid_est)

        # Sleep for a short duration to control the loop rate
        time.sleep(0.01)

    # Clean up and destroy the node
    nsaidEstimation.destroy_node()

    # Print a message indicating that the run has finished
    print("Run Finished")

    # Shut down the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
