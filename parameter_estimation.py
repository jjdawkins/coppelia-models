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



class parameterEstimation(Node):

    def __init__(self):
        super().__init__('param_estimation')

        # Define Known Parameters
        self.a = 0.167 # from CoppeliaSim
        self.b = 0.170 # from CoppeliaSim
        self.mu_f = 0.5
        self.mu_r = 0.4
        self.m = 4.378
        self.Jz = 0.0715

        # Define State Variables
        self.x = 0.0
        self.y = 0.0
        self.psi = 0.0
        self.r = 0.0
        self.u = 0.0
        self.v = 0.0
        self.beta = 0.0
        self.alpha_f = 0.0
        self.alpha_r = 0.0
        self.r_dot = 0.0
        self.u_dot = 0.0
        self.v_dot = 0.0
        self.u_des = 0.0
        self.delta = 0.0
        self.V_cmd = 0.0

        # Define Sensor Measurements
        self.r_meas = 0.0
        self.r_meas_old = 0.0
        self.ax_meas = 0.0
        self.ay_meas = 0.0
        self.f1_x = 0.0
        self.f1_y = 0.0
        self.f2_x = 0.0
        self.f2_y = 0.0

        # Define Estimate Variables
        self.r_dot_est = 0.0

        # Define time Variables
        (sec,nsec) = self.get_clock().now().seconds_nanoseconds()
        self.t_init = float(sec) + float(nsec*1e-9) 
        self.t_imu = 0.0
        self.t_imu_old = 0.0
        self.dt_imu = 0.05
        self.dt = 0.05
        self.imu_ind = 0

        # Initialize UKF
        accel_std = 0.1
        rdot_std = 0.1
        self.tau = 1
        self.w_sig = 0.0003
        Q_sig = 0.1
                
        self.P = np.diag([1,1,1,1])
        self.R = np.diag([accel_std**2,accel_std**2,rdot_std**2])    
        self.I = np.diag([1,1,1,1])
        self.H = np.zeros([3,4])
        self.x_hat = np.zeros([4,1])          
        

        self.plot_flag = False
        plt.ion()
        self.fig_imu,self.axs = plt.subplots(4,1)
        
        self.axs[0].set_ylabel('C_rr (m/s^2)')
        self.axs[1].set_ylabel('K_mot (m/s^2)')
        self.axs[2].set_ylabel('C_af (rad/s)')   
        self.axs[3].set_ylabel('C_af (rad/s)')            

        for i in range(len(self.axs)):  
            self.axs[i].plot(0,0)
            self.axs[i].set_xlabel('time (s)')

        self.odom_sub = self.create_subscription(Odometry,'/rover/mocap/odom',self.odom_callback,10)
        self.imu_sub = self.create_subscription(Imu,'/rover/imu',self.imu_callback,10)
        self.cmd_sub = self.create_subscription(Twist, '/rover/cmd_vel', self.cmd_vel_callback,10)
        self.f1_sub = self.create_subscription(Vector3,'/rover/forces_front',self.front_force_callback,10)
        self.f2_sub = self.create_subscription(Vector3,'/rover/forces_rear',self.rear_force_callback,10)


        self.target_pub = self.create_publisher(Pose,'/target_pose',10)
        self.dt = 0.05  # seconds     

        self.timer = self.create_timer(self.dt, self.run_loop)
    

    def front_force_callback(self,msg):
        self.f1_x = msg.x
        self.f1_y = msg.y

    def rear_force_callback(self,msg):
        self.f2_x = msg.x
        self.f2_y = msg.y

    def cmd_vel_callback(self,msg):
        self.delta = msg.angular.z
        self.V_cmd = msg.linear.x

    def imu_callback(self,msg):
        
        (sec,nsec) = self.get_clock().now().seconds_nanoseconds()
        
        self.t_imu = float(sec) + float(nsec*1e-9)-self.t_init        
        self.dt_imu = self.t_imu - self.t_imu_old
        self.ax_meas = msg.linear_acceleration.x
        self.ay_meas = msg.linear_acceleration.y
        self.r_meas = msg.angular_velocity.z
        # self.imu_data[:,self.imu_ind] = np.array([self.ax_meas,self.ay_meas,self.r_meas])
        self.imu_ind +=1

        self.r_dot_est = (self.r_meas - self.r_meas_old)/self.dt_imu
        self.plot_flag = True
        self.t_imu_old = self.t_imu
        self.r_meas_old = self.r_meas


    def odom_callback(self,msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quat = np.quaternion(msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z)
        R = quaternion.as_rotation_matrix(quat)
        eul = np.array([math.atan2(R[2][1],R[2][2]),-math.asin(R[2][0]),math.atan2(R[1][0],R[0][0])])            
        self.psi = eul[2]
        self.u = msg.twist.twist.linear.x
        self.v = msg.twist.twist.linear.y
        self.r = msg.twist.twist.angular.z

        if(math.fabs(self.u)>0.1):
            self.beta = math.atan(self.v/self.u)
            self.alpha_f = math.atan((self.v + self.a*self.r)/self.u) - self.delta
            self.alpha_r = math.atan((self.v + self.b*self.r)/self.u)

        else:
            self.beta = 0.0
            self.alpha_f = 0.0
            self.alpha_r = 0.0

    def update_plots(self):
        t_win = 20
        if(self.plot_flag):
            
            if(self.t_imu < t_win):
                t_min = 0.0
            else:
                t_min = self.t_imu - t_win

            lin, = self.axs[0].get_lines()
            lin.set_xdata(np.append(lin.get_xdata(),self.t_imu))
            lin.set_ydata(np.append(lin.get_ydata(),self.x_hat[0]))
            ymax = 1.2*np.max(lin.get_ydata())
            ymin = 1.2*np.min(lin.get_ydata())

            self.axs[0].set_xlim(left=t_min,right=self.t_imu)
            self.axs[0].set_ylim(top=ymax,bottom=ymin)
            
            lin, = self.axs[1].get_lines()
            lin.set_xdata(np.append(lin.get_xdata(),self.t_imu))
            lin.set_ydata(np.append(lin.get_ydata(),self.x_hat[1]))
            ymax = 1.2*np.max(lin.get_ydata())
            ymin = 1.2*np.min(lin.get_ydata())            
            self.axs[1].set_xlim(left=t_min,right=self.t_imu)
            self.axs[1].set_ylim(top=ymax,bottom=ymin)

            lin, = self.axs[2].get_lines()
            lin.set_xdata(np.append(lin.get_xdata(),self.t_imu))
            lin.set_ydata(np.append(lin.get_ydata(),self.x_hat[2]))
            ymax = 1.2*np.max(lin.get_ydata())
            ymin = 1.2*np.min(lin.get_ydata())  
            self.axs[2].set_xlim(left=t_min,right=self.t_imu)
            self.axs[2].set_ylim(top=ymax,bottom=ymin)


            lin, = self.axs[3].get_lines()
            lin.set_xdata(np.append(lin.get_xdata(),self.t_imu))
            lin.set_ydata(np.append(lin.get_ydata(),self.x_hat[3]))
            ymax = 1.2*np.max(lin.get_ydata())
            ymin = 1.2*np.min(lin.get_ydata())              
            self.axs[3].set_xlim(left=t_min,right=self.t_imu)
            self.axs[3].set_ylim(top=ymax,bottom=ymin)

            self.fig_imu.canvas.draw()             
            self.fig_imu.canvas.flush_events()


    def run_loop(self):


        if(math.fabs(self.u)>0.5): # Only update if moving to avoid singularity at u=0
            
            # Build Measurment from current Data
            zm = np.zeros([3,1])
            zm[0,0] = self.ax_meas - self.v*self.r_meas
            zm[1,0] = self.ay_meas + self.u*self.r_meas
            zm[2,0] = self.r_dot_est


            # Define Measurement Jacobian
            H23 = self.delta/self.m - self.v/(self.m*self.u)
            H24 = ((self.b*self.r)/(self.m*self.u)) - self.v/(self.m*self.u)

            H33 = self.a*self.delta/self.Jz - (self.a*self.a*self.r)/(self.Jz*self.u) - (self.a*self.v)/(self.Jz*self.u)
            H34 = (self.b*self.v)/(self.Jz*self.u) - (self.b*self.b*self.r)/(self.Jz*self.u)

            self.H[0,:] = np.array([-self.u/self.m, self.V_cmd/self.m, 0, 0])
            self.H[1,:] = np.array([0, 0, H23 , H24])
            self.H[2,:] = np.array([0, 0, H33, H34])        

            # Compute Residual
            y = zm - np.dot(self.H,self.x_hat)

            # Compute Kalman Gain
            S = np.linalg.inv(np.dot(np.dot(self.H,self.P),self.H.T) + self.R)
            K = np.dot(np.dot(self.P,self.H.T),S)

            # Update Parameter Estimate and Covariance
            self.x_hat = self.x_hat + np.dot(K,y)
            self.P = np.dot(self.I - np.dot(K,self.H),self.P)

            # Print current Parameter Estimates
            print(self.x_hat)


def ros_spin(node):
    rclpy.spin(node)
        

def main(args=None):
    rclpy.init(args=args)

    param_est = parameterEstimation()

    plt.ion()

    
    while(rclpy.ok):
        param_est.update_plots()

        rclpy.spin_once(param_est)
        time.sleep(0.01)

    param_est.destroy_node()
    print("Run Finished")
    rclpy.shutdown()


if __name__ == '__main__':
    main()