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

IDLE = -1
READY = 0
TAKEOFF = 1
HOVER = 2
AUTO = 3
MANUAL = 4
LAND = 5
ERROR = 6

def saturate(val,min,max):
    if(val > max):
        val = max

    if(val < min):
        val = min
    return val

class roverInterface(rclpy.node.Node):
    def __init__(self):
        super().__init__('drone_interface_node')

    #    print('NEW CODE')

        self.max_str = 0.35
        self.max_spd = 5.0
        self.str_cmd = 0.0
        self.spd_cmd = 0.0

        self.roll_joy = 0.0
        self.pitch_joy = 0.0
        self.yaw_joy = 0.0
        self.thrust_joy = 0.0


        self.flying = False        

        self.mode = IDLE

        #self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)

    #    self.declare_parameter('my_parameter', 'world')
        self.mav_conn = mavutil.mavlink_connection("/dev/ttyUSB0",baud=115200,input=False)

        self.imu_pub = self.create_publisher(Imu,'imu',10)

        self.set_sub = self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,1)
        self.joy_sub = self.create_subscription(Joy,'joy',self.joy_callback,1)
        #self.mocap_sub = self.create_subscription(Odometry,'mocap/odom',self.mocap_callback,1)

        self.timer = self.create_timer(0.01, self.send_loop)

        self.read_thread = threading.Thread(target=self.read_loop,daemon=True)
        self.read_thread.start()
        self.init_time = self.get_clock().now()
        self.last_rcvd = self.get_clock().now()
        self.imu_time = self.get_clock().now()

    #    print("NEW CODE")
    # def mocap_callback(self,msg):
    #     self.pos_x = self.pose.pose.position.x
    #     self.pos_y = self.pose.pose.position.y
    #     self.pos_z = self.pose.pose.position.z

    #     self.vel_x = self.twist.twist.linear.x
    #     self.vel_y = self.twist.twist.linear.y
    #     self.vel_z = self.twist.twist.linear.z




    def send_loop(self):

        time_from_boot = self.get_clock().now() - self.init_time
        time = int(time_from_boot.nanoseconds*1e-9)
        
        dt = self.get_clock().now()-self.last_rcvd

        str_pwm = 1500
        spd_pwm = 1500
        if(dt.nanoseconds*1e-6 < 500):
            str_pwm = int(self.str_cmd*600 + 1500)
            spd_pwm = int(self.spd_cmd*500 + 1500)
        else:
            pass


        self.mav_conn.mav.rc_channels_override_send(
              target_system=1,  # Replace with the target system ID
              target_component=1,  # Replace with the target component ID
              chan1_raw=str_pwm,
              chan2_raw=spd_pwm,
              chan3_raw=0,
              chan4_raw=0,
              chan5_raw=0,  # You can set these to 0 if not used
              chan6_raw=0,
              chan7_raw=0,
              chan8_raw=0
         )
        
        #self.mav_conn.mav.manual_setpoint_send(time_ms,roll_cmd,pitch_cmd,yaw_cmd,thrust_cmd, mode_cmd,0)            

    def joy_callback(self,msg):

        self.roll_joy = (-msg.axes[3])*self.max_roll
        self.pitch_joy = msg.axes[4]*self.max_pitch
        self.yaw_joy = msg.axes[0]*self.max_yawrate
        self.thrust_joy = msg.axes[1]
        self.thrust_joy = saturate(self.thrust_joy,0.0,1.0)

        if(msg.buttons[0]):     # A Button
            pass

        if(msg.buttons[1]):     # B Button
            pass

        if(msg.buttons[2]):     # X Button
            if(self.flying):
                self.mode = LAND
            pass

        if(msg.buttons[3]):     # Y Button
            if(not self.flying):
                self.mode = TAKEOFF


        if(msg.buttons[4]):     # L Button
            if(self.flying):
                self.mode = AUTO
            pass

        if(msg.buttons[5]):     # R Button
            #if(self.flying):
            self.mode = MANUAL
                                        


    def cmd_vel_callback(self,msg):

        self.last_rcvd = self.get_clock().now()
        #time_from_boot = self.get_clock().now() - self.init_time
        #time_ms = int(time_from_boot.nanoseconds*1e-6)
        

        self.str_cmd = msg.angular.z/self.max_str
        self.spd_cmd = msg.linear.x/self.max_spd

        self.str_cmd = saturate(self.str_cmd,-self.max_str,self.max_str)
        self.spd_cmd = saturate(self.spd_cmd,-self.max_spd,self.max_spd)

    def read_loop(self):
        
        while(True):

            #msg = self.ser.readline()
            msg = self.mav_conn.recv_match(blocking=True)

            #print(msg)
            if(msg is None):
                pass
            elif(msg.get_type() == 'HEARTBEAT'):
                pass
            elif(msg.get_type() == 'SCALED_IMU'):
                #print((1e-6)*(self.get_clock().now()-self.imu_time).nanoseconds)
                self.imu_time = self.get_clock().now()
                
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.linear_acceleration.x = msg.xacc*1e-3
                imu_msg.linear_acceleration.y = msg.yacc*1e-3
                imu_msg.linear_acceleration.z = msg.zacc*1e-3
                imu_msg.angular_velocity.x = msg.xgyro*1e-3
                imu_msg.angular_velocity.y = msg.ygyro*1e-3
                imu_msg.angular_velocity.z = msg.zgyro*1e-3

                self.imu_pub.publish(imu_msg)                                
            time.sleep(0.0001)


def main():
    rclpy.init()
    node = roverInterface()
    print("newCode")
    rclpy.spin(node)

if __name__ == '__main__':
    main()
