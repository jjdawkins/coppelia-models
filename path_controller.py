import rclpy
import numpy as np
import quaternion
import math
import time
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, Pose

# Define a class for the rover path controller


class roverPathControl(Node):
    """
    Class for controlling the path of a rover.
    """

    def __init__(self):
        """
        Initialize the roverPathControl class.
        """
        super().__init__("path_controller")

        # Controller gains
        self.K_psi = 1.0
        self.K_ct = 1.0
        self.K_spd = 0.4
        self.cruise_spd = 1.0
        self.max_speed = 3.5
        self.max_steer = 0.35

        # Rover state variables
        self.x = 0.0
        self.y = 0.0
        self.psi = 0.0
        self.r = 0.0
        self.u = 0.0
        self.v = 0.0
        self.mission = False

        # Create subscribers and publishers
        self.path_sub = self.create_subscription(
            Path, "/path", self.path_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/rover/mocap/odom", self.odom_callback, 10
        )
        self.target_pub = self.create_publisher(Pose, "/target_pose", 10)
        self.cmd_pub = self.create_publisher(Twist, "/rover/cmd_vel", 10)

        # Control loop parameters
        self.dt = 0.05  # seconds
        self.timer = self.create_timer(self.dt, self.control_loop)

    def wrapToPi(self, ang):
        """
        Helper function to wrap an angle to the range [-pi, pi].

        Args:
            ang (float): The angle to be wrapped.

        Returns:
            float: The wrapped angle.
        """
        while ang > math.pi:
            ang = ang - 2 * math.pi

        while ang < -math.pi:
            ang = ang + 2 * math.pi

        return ang

    def findClosestPoint(self, path_pose, pose):
        """
        Helper function to find the index of the closest point on the path.

        Args:
            path_pose (numpy.ndarray): Array of path poses.
            pose (numpy.ndarray): Current pose.

        Returns:
            int: Index of the closest point on the path.
        """
        diff = path_pose - pose
        rng = np.sqrt(diff[:, 0] * diff[:, 0] + diff[:, 1] * diff[:, 1])
        index = np.argmin(rng)
        return index

    def findPathLocation(self, path_s, car_s):
        """
        Helper function to find the index of the path location closest to the rover.

        Args:
            path_s (numpy.ndarray): Array of path locations.
            car_s (float): Current rover location.

        Returns:
            int: Index of the path location closest to the rover.
        """
        diff = path_s - car_s
        index = np.argmin(np.abs(diff))
        return index

    def saturate(self, val, min, max):
        """
        Helper function to saturate a value between a minimum and maximum.

        Args:
            val (float): The value to be saturated.
            min (float): The minimum value.
            max (float): The maximum value.

        Returns:
            float: The saturated value.
        """
        if val > max:
            val = max
        if val < min:
            val = min

        return val

    def path_callback(self, msg):
        """
        Callback function for the path subscriber.

        Args:
            msg: The received path message.
        """
        print("Path Received")
        self.wp_n = len(msg.poses)
        self.path = np.zeros([self.wp_n, 3])
        self.path_s = np.zeros([self.wp_n])
        self.s = 0

        for i in range(self.wp_n):
            # Extract quaternion from pose message
            orientation_q = msg.poses[i].pose.orientation
            quat = np.quaternion(
                msg.poses[i].pose.orientation.w,
                msg.poses[i].pose.orientation.x,
                msg.poses[i].pose.orientation.y,
                msg.poses[i].pose.orientation.z,
            )
            R = quaternion.as_rotation_matrix(quat)
            eul = np.array(
                [
                    math.atan2(R[2][1], R[2][2]),
                    -math.asin(R[2][0]),
                    math.atan2(R[1][0], R[0][0]),
                ]
            )

            self.path[i,0] = msg.poses[i].pose.position.x
            self.path[i,1] = msg.poses[i].pose.position.y
            # self.path[i,2] = eul[2]

        dx = np.roll(self.path[:,0],-1)-self.path[:,0]
        dy = np.roll(self.path[:,1],-1)-self.path[:,1]
        dpsi = np.arctan2(dy,dx)

        self.path[:,2] = dpsi
        #dx = np.diff(self.path[:,0])
        #dy = np.diff(self.path[:,1])
        r = np.sqrt(dx*dx + dy*dy)

        print(self.path)
        # print(len(np.cumsum(r)))

        self.path_s[0:len(self.path)] = np.cumsum(r)
        self.path_l = self.path_s[np.size(r)-1]
        # print(self.path_l)
        self.mission = True

        # print size of received path

    def odom_callback(self, msg):
        """
        Callback function for the odometry subscriber.

        Args:
            msg: The received odometry message.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quat = np.quaternion(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        )
        R = quaternion.as_rotation_matrix(quat)
        eul = np.array(
            [
                math.atan2(R[2][1], R[2][2]),
                -math.asin(R[2][0]),
                math.atan2(R[1][0], R[0][0]),
            ]
        )
        self.psi = eul[2]
        self.u = msg.twist.twist.linear.x
        self.v = msg.twist.twist.linear.y
        self.r = msg.twist.twist.angular.z

    def control_loop(self):
        """
        Control loop function.
        """
        if self.mission:
            if (
                self.s > self.path_l
            ):  # If we reach the end of the path start back at zero
                self.s = 0
                self.cruise_spd += 0.5
                if self.cruise_spd > self.max_speed:
                    self.cruise_spd = 1.0
            else:
                self.desSpeed = self.cruise_spd

            self.s = self.s + self.cruise_spd * self.dt

            self.xdes = np.interp(self.s, self.path_s, self.path[:, 0])
            self.ydes = np.interp(self.s, self.path_s, self.path[:, 1])
            self.psi_path = np.interp(self.s, self.path_s, self.path[:, 2])

            ind = self.findPathLocation(self.path_s, self.s)

            self.xerr = self.xdes - self.x
            self.yerr = self.ydes - self.y

            tgt_pose_msg = Pose()
            tgt_pose_msg.position.x = self.xdes
            tgt_pose_msg.position.y = self.ydes
            self.target_pub.publish(tgt_pose_msg)

            self.psi_des = math.atan2(self.yerr, self.xerr)

            self.xerr_b = (
                math.cos(self.psi) * self.xerr + math.sin(self.psi) * self.yerr
            )
            self.yerr_b = (
                -math.sin(self.psi) * self.xerr +
                math.cos(self.psi) * self.yerr
            )

            self.psi_err = self.wrapToPi(self.psi_path - self.psi)

            str_ang = 0.0
            if abs(self.u)<0.1: # if speed less than 10 cm/s, use speed invariant turn angle
                str_ang = self.psi_err 
            else: # if speed greater than 10 cm/s, use speed dependent steering angle
                str_ang = self.K_psi*self.psi_err/self.u + self.K_ct*math.atan(self.yerr_b/self.u)

            str_ang = self.saturate(str_ang, -self.max_steer, self.max_steer)
            spd_cmd = self.cruise_spd + self.K_spd * self.xerr_b

            spd_cmd = self.saturate(spd_cmd, -self.max_speed, self.max_speed)

            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = spd_cmd
            cmd_vel_msg.angular.z = str_ang

            self.cmd_pub.publish(cmd_vel_msg)
        else:
            print("Waiting for Mission")
            time.sleep(1)


def main(args=None):
    rclpy.init(args=args)

    path_ctrl = roverPathControl()

    rclpy.spin(path_ctrl)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_ctrl.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
