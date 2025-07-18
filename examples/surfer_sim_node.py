# !/usr/bin/env python3

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
import numpy as np

class SurferSim(Node):
    def __init__(self):
        super().__init__('surfer_sim')


        self.name = 'surfer'

        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        self.sim.setStepping(True)
        self.cmd_vel_timer = self.get_clock().now()
        self.sim_status = 'stopped'
        self.cmd = np.array([0.0, 0.0, 0.0])  # Initialize command array
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            f'/{self.name}/cmd_vel', 
            self.cmd_vel_callback, 
            10
        )

        self.gamma = np.array([
                            [ 0.0,  -1.0,  -1.56],
                            [-1.0,   0.0,  -1.56],
                            [ 0.0,   1.0,  -1.56],
                            [ 1.0,   0.0,  -1.56]
        ])  # Gamma matrix for control

        self.sim_time_pub = self.create_publisher(Clock, '/clock', 10)


        self.create_timer(0.05, self.loop_callback)
        self.get_logger().info('Surfer Simulation Node has been initialized.')

    def get_handles(self):
        self.surfer_h = self.sim.getObjectHandle(self.name)

    def cmd_vel_callback(self, msg):
        # Convert Twist message to CoppeliaSim command
        self.cmd[0] = msg.linear.x  # Forward/backward velocity
        self.cmd[1] = msg.linear.y  # Left/right velocity (if applicable)
        self.cmd[2] = msg.angular.z  # Angular velocity
        #self.cmd = np.clip(self.cmd, -1.0, 1.0)  # Limit command values to [-1, 1]
        self.cmd_vel_timer = self.get_clock().now()  # Update the timer to indicate command received


    def loop_callback(self):

        sim_state = self.sim.getSimulationState()

        if (self.get_clock().now() - self.cmd_vel_timer).nanoseconds > 3e8:  # If no command received for 0.3 seconds
            self.cmd = np.array([0.0, 0.0, 0.0])  # Stop the robot if no command is received
#
        if(sim_state == self.sim.simulation_advancing_running):

            # self.cmd[2] = 2 * self.cmd[2]  # Scale angular velocity for better control
            self.cmd_motors = self.gamma @ self.cmd
            motor_pwm = 1500 + 400 * self.cmd_motors
            motor_pwm = np.clip(motor_pwm, 1100, 1900)
            print(f'Motor PWM: {motor_pwm}')

            self.sim.setFloatSignal(f'thruster_1_pwm', motor_pwm[0])
            self.sim.setFloatSignal(f'thruster_2_pwm', motor_pwm[1])
            self.sim.setFloatSignal(f'thruster_3_pwm', motor_pwm[2])
            self.sim.setFloatSignal(f'thruster_4_pwm', motor_pwm[3])

            self.sim.step()



    def start_simulation_callback(self, request, response):
        
        if self.sim_status == 'stopped':
            # Add code to start CoppeliaSim simulation here
            self.get_logger().info('Starting CoppeliaSim simulation...')
            self.sim.startSimulation()

            response.success = True
            response.message = 'Simulation started.'
        else:
            response.success = False
            response.message = 'Simulation already running.'

        return response

    # def pause_simulation_callback(self, request, response):
    #     if self.sim_status == 'running':
    #         # Add code to pause CoppeliaSim simulation here
    #         self.get_logger().info('Pausing CoppeliaSim simulation...')
    #         self.sim.pauseSimulation()
    #         self.sim_status = 'paused'
    #         response.success = True
    #         response.message = 'Simulation paused.'
    #     else:
    #         response.success = False
    #         response.message = 'Simulation not running.'
    #     return response
    
    def stop_simulation_callback(self, request, response):
        if self.sim_status == 'running':
            # Add code to stop CoppeliaSim simulation here
            self.sim.stopSimulation()
            self.sim_status = 'stopped'
            response.success = True
            response.message = 'Simulation stopped.'
        else:
            response.success = False
            response.message = 'Simulation not running.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SurferSim()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()