#!/bin/python3

import math
import rclpy
import rclpy.node
import threading
import time
from pymavlink import mavutil

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

import serial

# Define states for the rover
IDLE = -1
READY = 0
TAKEOFF = 1
HOVER = 2
AUTO = 3
MANUAL = 4
LAND = 5
ERROR = 6


# Helper function to constrain values within a range
def saturate(val, min, max):
    if val > max:
        val = max
    if val < min:
        val = min
    return val


class roverInterface(rclpy.node.Node):
    def __init__(self):
        # Initialize the node
        super().__init__("drone_interface_node")

        # Initialize parameters
        self.max_str = 0.35
        self.max_spd = 15.0
        self.str_cmd = 0.0
        self.spd_cmd = 0.0

        self.roll_joy = 0.0
        self.pitch_joy = 0.0
        self.yaw_joy = 0.0
        self.thrust_joy = 0.0

        self.flying = False
        self.mode = IDLE

        # Setup MAVLink connection
        # Uncomment if using serial connection: self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
        self.mav_conn = mavutil.mavlink_connection(
            "/dev/ttyUSB0", baud=115200, input=False
        )

        # Setup publishers and subscribers
        self.imu_pub = self.create_publisher(Imu, "/rover/imu", 10)
        self.set_sub = self.create_subscription(
            Twist, "/rover/cmd_vel", self.cmd_vel_callback, 1
        )
        self.joy_sub = self.create_subscription(Joy, "/rover/joy", self.joy_callback, 1)
        # Uncomment if using motion capture data: self.mocap_sub = self.create_subscription(Odometry, 'mocap/odom', self.mocap_callback, 1)

        # Timer for periodic tasks
        self.timer = self.create_timer(0.01, self.send_loop)

        # Start a background thread for reading MAVLink messages
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

        # Initialize timing
        self.init_time = self.get_clock().now()
        self.last_rcvd = self.get_clock().now()
        self.imu_time = self.get_clock().now()

    # Function to handle sending commands via MAVLink
    def send_loop(self):
        time_from_boot = self.get_clock().now() - self.init_time
        time = int(time_from_boot.nanoseconds * 1e-9)

        dt = self.get_clock().now() - self.last_rcvd

        # Calculate PWM values based on commands
        str_pwm = 1500
        spd_pwm = 1500

        # only send commands if the cmd has been received recently
        if dt.nanoseconds * 1e-9 < 0.4:
            str_pwm = int(self.str_cmd * 500 + 1500)  # STEERING from 0 to 1
            spd_pwm = int(self.spd_cmd * 500 + 1500)
            #print(f"sending {spd_pwm}")



        # Send the commands via MAVLink
        self.mav_conn.mav.rc_channels_override_send(
            target_system=1,  # Replace with the target system ID
            target_component=1,  # Replace with the target component ID
            chan1_raw=str_pwm,
            chan2_raw=spd_pwm,
            chan3_raw=0,
            chan4_raw=0,
            chan5_raw=0,
            chan6_raw=0,
            chan7_raw=0,
            chan8_raw=0,
        )

    # Function to handle joystick inputs
    def joy_callback(self, msg):
        self.roll_joy = (-msg.axes[3]) * self.max_roll
        self.pitch_joy = msg.axes[4] * self.max_pitch
        self.yaw_joy = msg.axes[0] * self.max_yawrate
        self.thrust_joy = msg.axes[1]
        self.thrust_joy = saturate(self.thrust_joy, 0.0, 1.0)

        # Handle joystick button presses
        if msg.buttons[0]:  # A Button
            pass

        if msg.buttons[1]:  # B Button
            pass

        if msg.buttons[2]:  # X Button
            if self.flying:
                self.mode = LAND

        if msg.buttons[3]:  # Y Button
            if not self.flying:
                self.mode = TAKEOFF

        if msg.buttons[4]:  # L Button
            if self.flying:
                self.mode = AUTO

        if msg.buttons[5]:  # R Button
            self.mode = MANUAL

    # Function to handle Twist messages
    def cmd_vel_callback(self, msg):
        self.last_rcvd = self.get_clock().now()

        # Normalize and constrain commands
        self.str_cmd = msg.angular.z
        self.spd_cmd = msg.linear.x
        # self.str_cmd = saturate(self.str_cmd, -self.max_str, self.max_str)
        # self.spd_cmd = saturate(self.spd_cmd, -self.max_spd, self.max_spd)

    # Function to read MAVLink messages
    def read_loop(self):
        while True:
            # Read MAVLink message
            msg = self.mav_conn.recv_match(blocking=True)

            if msg is None:
                pass
            elif msg.get_type() == "HEARTBEAT":
                pass
            elif msg.get_type() == "SCALED_IMU":
                # Process IMU data
                self.imu_time = self.get_clock().now()

                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.linear_acceleration.x = msg.xacc * 1e-3
                imu_msg.linear_acceleration.y = msg.yacc * 1e-3
                imu_msg.linear_acceleration.z = msg.zacc * 1e-3
                imu_msg.angular_velocity.x = msg.xgyro * 1e-3
                imu_msg.angular_velocity.y = msg.ygyro * 1e-3
                imu_msg.angular_velocity.z = msg.zgyro * 1e-3

                self.imu_pub.publish(imu_msg)
            time.sleep(0.0001)


def main():
    rclpy.init()
    node = roverInterface()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
