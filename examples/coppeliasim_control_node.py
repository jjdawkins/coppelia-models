# !/usr/bin/env python3

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock

class CoppeliaSimController(Node):
    def __init__(self):
        super().__init__('coppelia_sim_controller')


        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        self.sim.setStepping(True)
        self.sim_status = 'stopped'

        self.start_srv = self.create_service(Trigger, 'start_simulation', self.start_simulation_callback)
        # self.pause_srv = self.create_service(Trigger, 'pause_simulation', self.pause_simulation_callback)
        self.stop_srv = self.create_service(Trigger, 'stop_simulation', self.stop_simulation_callback)

        self.sim_time_pub = self.create_publisher(Clock, '/clock', 10)

        # self.sim.startSimulation()
        # time.sleep(5)  # Allow some time for the simulation to start
        # self.sim.stopSimulation()

        self.create_timer(0.05, self.publish_sim_time)
        self.get_logger().info('CoppeliaSim Controller Node has been initialized.')
        
    def publish_sim_time(self):
        #self.get_logger().info(f'Waiting Sim Status {self.sim_running}...')
        sim_state = self.sim.getSimulationState()
        print(f'Simulation State: {sim_state}')

        states = [self.sim.simulation_stopped,
                   self.sim.simulation_paused,
                   self.sim.simulation_advancing_firstafterstop,
                   self.sim.simulation_advancing_running,
                   self.sim.simulation_advancing_lastbeforepause,
                   self.sim.simulation_advancing_firstafterpause,
                   self.sim.simulation_advancing_abouttostop,
                   self.sim.simulation_advancing_lastbeforestop]

        print(f'Simulation States: {states}')

        if sim_state == 0:
            self.sim_status = 'stopped'

        elif sim_state == 17:
            self.sim_status = 'running'

        elif sim_state == 8:
            self.sim_status = 'paused'
            self.get_logger().info('Simulation is paused.')
            self.sim_running = False
        else:
            pass


        if self.sim_status == 'running':
            sim_time = self.sim.getSimulationTime()
            sec = int(sim_time)
            nanosec = int((sim_time - sec) * 1e9)
            msg = Clock()
            msg.clock.sec = sec
            msg.clock.nanosec = nanosec
            self.sim.step()
            self.sim_time_pub.publish(msg)
        else:
            #self.get_logger().info('Simulation is not running, skipping time publication.')
            pass

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
    node = CoppeliaSimController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()