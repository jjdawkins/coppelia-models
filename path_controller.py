import rclpy
import numpy as np
import quaternion
import math
import time
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, Pose



class roverPathControl(Node):
    """
    Class for controlling the path of a rover.

    Attributes:
        K_psi (float): Proportional gain for yaw angle control.
        K_ct (float): Proportional gain for lateral error control.
        K_spd (float): Proportional gain for speed control.
        cruise_spd (float): Desired cruising speed.
        max_speed (float): Maximum speed of the rover.
        max_steer (float): Maximum steering angle of the rover.
        x (float): Current x-coordinate of the rover.
        y (float): Current y-coordinate of the rover.
        psi (float): Current yaw angle of the rover.
        r (float): Current yaw rate of the rover.
        u (float): Current x-velocity of the rover.
        v (float): Current y-velocity of the rover.
        mission (bool): Flag indicating if a mission is in progress.
        path_sub (Subscription): Subscription to the path topic.
        odom_sub (Subscription): Subscription to the odometry topic.
        target_pub (Publisher): Publisher for the target pose topic.
        cmd_pub (Publisher): Publisher for the command velocity topic.
        dt (float): Time step for the control loop.
        timer (Timer): Timer for the control loop.
    """

    def __init__(self):
        """
        Initializes the roverPathControl object.
        """
        super().__init__('path_controller')
        # Controller gains
        self.K_psi = 1.0  # Proportional gain for yaw angle control
        self.K_ct = 1.0  # Proportional gain for lateral error control
        self.K_spd = 0.2  # Proportional gain for speed control

        # Rover parameters
        self.cruise_spd = 1.0  # Desired cruising speed
        self.max_speed = 3.5  # Maximum speed of the rover
        self.max_steer = 0.35  # Maximum steering angle of the rover

        # Rover state variables
        self.x = 0.0  # Current x-coordinate of the rover
        self.y = 0.0  # Current y-coordinate of the rover
        self.psi = 0.0  # Current yaw angle of the rover
        self.r = 0.0  # Current yaw rate of the rover
        self.u = 0.0  # Current x-velocity of the rover
        self.v = 0.0  # Current y-velocity of the rover
        self.mission = False  # Flag indicating if a mission is in progress

        # Subscriptions
        self.path_sub = self.create_subscription(Path, '/path', self.path_callback, 10)  # Subscription to the path topic
        self.odom_sub = self.create_subscription(Odometry, '/rover/mocap/odom', self.odom_callback, 10)  # Subscription to the odometry topic

        # Publishers
        self.target_pub = self.create_publisher(Pose, '/target_pose', 10)  # Publisher for the target pose topic
        self.cmd_pub = self.create_publisher(Twist, '/rover/cmd_vel', 10)  # Publisher for the command velocity topic

        # Control loop parameters
        self.dt = 0.05  # Time step for the control loop
        self.timer = self.create_timer(self.dt, self.control_loop)  # Timer for the control loop

    def wrapToPi(self, ang):
        """
        Wraps an angle to the range [-pi, pi].

        Args:
            ang (float): Angle to be wrapped.

        Returns:
            float: Wrapped angle.
        """
        ang = ang % (2*math.pi)

        return ang

    def findClosestPoint(self, path_pose, pose):
        """
        Finds the index of the closest point on the path to the given pose.

        Args:
            path_pose (numpy.ndarray): Array of path poses.
            pose (float): Current pose.

        Returns:
            int: Index of the closest point.
        """
        diff = path_pose - pose
        rng = np.sqrt(diff[:,0]*diff[:,0]+diff[:,1]*diff[:,1])
        index = np.argmin(rng)
        return index

    def findPathLocation(self, path_s, car_s):
        """
        Finds the index of the path location closest to the given car location.

        Args:
            path_s (numpy.ndarray): Array of path locations.
            car_s (float): Current car location.

        Returns:
            int: Index of the path location.
        """
        diff = path_s - car_s
        index = np.argmin(np.abs(diff))
        return index

    def saturate(self, val, min_val, max_val):
        """
        Saturates a value between a minimum and maximum value.

        Args:
            val (float): Value to be saturated.
            min_val (float): Minimum value.
            max_val (float): Maximum value.

        Returns:
            float: Saturated value.
        """
        if val > max_val:
            val = max_val
        if val < min_val:
            val = min_val

        return val

    def path_callback(self, msg):
        """
        Callback function for the path topic.

        Args:
            msg: Path message.
        """
        print('Path Received')  # Print a message indicating that the path has been received
        self.wp_n = len(msg.poses)  # Get the number of waypoints in the path
        self.path = np.zeros([self.wp_n,3])  # Initialize an array to store the path coordinates and yaw angles
        self.path_s = np.zeros([self.wp_n])  # Initialize an array to store the path locations
        self.s = 0  # Initialize the current path location

        for i in range(self.wp_n):
            # Extract quaternion from pose message
            orientation_q = msg.poses[i].pose.orientation
            quat = np.quaternion(msg.poses[i].pose.orientation.w, msg.poses[i].pose.orientation.x,
                                 msg.poses[i].pose.orientation.y, msg.poses[i].pose.orientation.z)
            R = quaternion.as_rotation_matrix(quat)
            eul = np.array([math.atan2(R[2][1], R[2][2]), -math.asin(R[2][0]), math.atan2(R[1][0], R[0][0])])

            self.path[i,0] = msg.poses[i].pose.position.x  # Store the x-coordinate of the waypoint
            self.path[i,1] = msg.poses[i].pose.position.y  # Store the y-coordinate of the waypoint
            self.path[i,2] = eul[2]  # Store the yaw angle of the waypoint

        dx = np.diff(self.path[:,0])  # Calculate the differences in x-coordinates between waypoints
        dy = np.diff(self.path[:,1])  # Calculate the differences in y-coordinates between waypoints
        r = np.sqrt(dx*dx + dy*dy)  # Calculate the distances between waypoints

        self.path_s[1:len(self.path)] = np.cumsum(r)  # Calculate the cumulative sum of the distances to get the path locations
        self.path_l = self.path_s[np.size(r)-1]  # Get the total length of the path
        self.mission = True  # Set the mission flag to indicate that a mission is in progress

    def odom_callback(self, msg):
        """
        Callback function for the odometry topic.

        Args:
            msg: Odometry message.
        """
        self.x = msg.pose.pose.position.x  # Update the current x-coordinate of the rover
        self.y = msg.pose.pose.position.y  # Update the current y-coordinate of the rover
        quat = np.quaternion(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                             msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)
        R = quaternion.as_rotation_matrix(quat)
        eul = np.array([math.atan2(R[2][1], R[2][2]), -math.asin(R[2][0]), math.atan2(R[1][0], R[0][0])])
        self.psi = eul[2]  # Update the current yaw angle of the rover
        self.u = msg.twist.twist.linear.x  # Update the current x-velocity of the rover
        self.v = msg.twist.twist.linear.y  # Update the current y-velocity of the rover
        self.r = msg.twist.twist.angular.z  # Update the current yaw rate of the rover

    def control_loop(self):
        """
        Control loop for the rover path controller.
        """
        if self.mission:
            if self.s > self.path_l:
                self.s = 0  # Reset the current path location if it exceeds the total length of the path
                self.cruise_spd += 0.5  # Increase the cruising speed by 0.5
                if self.cruise_spd > self.max_speed:
                    self.cruise_spd = 1.0  # Reset the cruising speed to 1.0 if it exceeds the maximum speed
            else:
                self.desSpeed = self.cruise_spd  # Set the desired speed to the cruising speed

            self.s = self.s + self.cruise_spd*self.dt  # Update the current path location based on the cruising speed

            self.xdes = np.interp(self.s, self.path_s, self.path[:,0])  # Interpolate the desired x-coordinate based on the current path location
            self.ydes = np.interp(self.s, self.path_s, self.path[:,1])  # Interpolate the desired y-coordinate based on the current path location
            self.psi_path = np.interp(self.s, self.path_s, self.path[:,2])  # Interpolate the desired yaw angle based on the current path location

            ind = self.findPathLocation(self.path_s, self.s)  # Find the index of the path location closest to the current path location

            self.xerr = self.xdes - self.x  # Calculate the x-error between the desired and current x-coordinates
            self.yerr = self.ydes - self.y  # Calculate the y-error between the desired and current y-coordinates

            tgt_pose_msg = Pose()
            tgt_pose_msg.position.x = self.xdes  # Set the desired x-coordinate in the target pose message
            tgt_pose_msg.position.y = self.ydes  # Set the desired y-coordinate in the target pose message
            self.target_pub.publish(tgt_pose_msg)  # Publish the target pose message

            self.psi_des = math.atan2(self.yerr, self.xerr)  # Calculate the desired yaw angle based on the x and y errors

            self.xerr_b = math.cos(self.psi) * self.xerr + math.sin(self.psi) * self.yerr  # Transform the x-error to the body frame
            self.yerr_b = -math.sin(self.psi) * self.xerr + math.cos(self.psi) * self.yerr  # Transform the y-error to the body frame

            self.psi_err = self.wrapToPi(self.psi_path - self.psi)  # Calculate the yaw angle error

            str_ang = 0.0
            if abs(self.u) < 0.1:
                str_ang = self.psi_err  # Set the steering angle to the yaw angle error if the x-velocity is small
            else:
                str_ang = self.K_psi * self.psi_err + math.atan(self.K_ct * self.yerr_b / self.u)  # Calculate the steering angle based on the yaw angle error and lateral error

            str_ang = self.saturate(str_ang, -self.max_steer, self.max_steer)  # Saturate the steering angle within the maximum steering angle limits
            spd_cmd = self.cruise_spd + self.K_spd * self.xerr_b  # Calculate the speed command based on the cruising speed and x-error in the body frame

            spd_cmd = self.saturate(spd_cmd, -self.max_speed, self.max_speed)  # Saturate the speed command within the maximum speed limits

            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = spd_cmd  # Set the linear velocity in the command velocity message
            cmd_vel_msg.angular.z = str_ang  # Set the angular velocity in the command velocity message

            self.cmd_pub.publish(cmd_vel_msg)  # Publish the command velocity message
        else:
            print("Waiting for Mission")  # Print a message indicating that the rover is waiting for a mission
            time.sleep(1)  # Sleep for 1 second

def main(args=None):
    rclpy.init(args=args)

    path_ctrl = roverPathControl()

    rclpy.spin(path_ctrl)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_ctrl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()