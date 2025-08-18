# !/usr/bin/env python3

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import signal
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3D, Detection3DArray
import numpy as np

import json
import cv2

class OakSim(Node):
    def __init__(self):
        super().__init__('oak_sim')


        self.name = 'oak'

        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        # self.sim.setStepping(True)
        self.cmd_vel_timer = self.get_clock().now()
        self.sim_status = 'stopped'

        cv2.namedWindow('Oak RGB Image', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Oak Depth Image', cv2.WINDOW_NORMAL)

        self.image_pub = self.create_publisher(Image, f'/{self.name}/image', 10)


        self.sim_time_pub = self.create_publisher(Clock, '/clock', 10)

        self.get_handles()
        self.create_timer(0.005, self.loop_callback)
        self.get_logger().info('Surfer Simulation Node has been initialized.')

    def get_handles(self):
        # self.surfer_h = self.sim.getObjectHandle(f"/{self.name}")
        # self.imu_h = self.sim.getObjectHandle(':/imu')
        self.oak_h = self.sim.getObjectHandle(':/oak_sensor')
        self.clip_planes = self.sim.getProperty(self.oak_h, "clippingPlanes")
        print(self.clip_planes)


    def loop_callback(self):

        sim_state = self.sim.getSimulationState()
#
        if(sim_state == self.sim.simulation_advancing_running):


            img_buf, res = self.sim.getVisionSensorImg(self.oak_h, 0, 0.0, [0, 0], [0, 0])
            depth_buf,res = self.sim.getVisionSensorDepth(self.oak_h, 0, [0, 0], [0, 0])
            depth = np.frombuffer(depth_buf, dtype=np.float32)

            depth_img = (depth).reshape(res[1], res[0], 1)
            # depth_img = cv2.resize(depth_img, (640, 480))  # Resize to desired dimensions

            depth_img = (255*depth_img).astype(np.uint8)  # Scale depth values to 0-255 for visualization

            image = np.frombuffer(img_buf, dtype=np.uint8)
            image = image.reshape(res[1], res[0], 3)  # Reshape to height, width, channels
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
            # image = cv2.resize(image, (640, 480))  # Resize to desired dimensions
            image = cv2.flip(image, 1)  # Flip the image vertically if needed
            
            # Convert image to HSV color space for better color segmentation
            hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)



            lower_red1 = np.array([100, 70, 50])
            upper_red1 = np.array([179, 255, 255])
            # Define lower and upper bounds for green color in HSV
            lower_green = np.array([40, 70, 50])
            upper_green = np.array([80, 255, 255])

            # Create masks for red color (covering both hue ranges)
            mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)

            red_mask = cv2.inRange(hsv_image, lower_red1, upper_red1)
            red_mask = cv2.erode(red_mask, None, iterations=1)  # Erode to reduce noise
            red_mask = cv2.dilate(red_mask, None, iterations=2)  # Dilate to enhance the red regions

            grn_mask = cv2.inRange(hsv_image, lower_green, upper_green)
            grn_mask = cv2.erode(grn_mask, None, iterations=1)  # Erode to reduce noise
            grn_mask = cv2.dilate(grn_mask, None, iterations=2)  # Dilate to enhance the green regions


            # Find contours (blobs) in the mask
            red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            grn_contours, _ = cv2.findContours(grn_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            bouys = []
            detections_msg = Detection3DArray()
            for cnt in red_contours:
                # area = cv2.contourArea(cnt)
                # if area > 5:  # Filter small blobs
                detection = Detection3D()
                x, y, w, h = cv2.boundingRect(cnt)
                box = [x, y, w, h]
                # Find the corresponding values of depth image to the contours
                # Extract the region of interest (ROI) from the depth image using the bounding rectangle
                roi = depth_img[y:y+h, x:x+w]
                # Calculate the average depth in the ROI
                avg_depth = (self.clip_planes[1]-self.clip_planes[0])*(np.mean(roi)/255) if roi.size > 0 else 0 

                print(f"Bouy at ({x + w // 2}, {y + h // 2}) with average depth: {avg_depth:.2f} m")    

                cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 1)
                # bouys.append((x + w // 2, y + h // 2))

            for cnt in grn_contours:
                # area = cv2.contourArea(cnt)
                # if area > 5:  # Filter small blobs
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 1)
                # cv2.drawContours(image, [cnt], -1, (255, 0, 0), 2)
            
            cv2.imshow('Oak RGB Image', image)
            cv2.waitKey(1)
            # print(image.shape)

            # print(camera_info_data)  # Check if image buffer is not empty

            # self.sim.step()

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

    def cleanup(self):
        """Gracefully cleanup CoppeliaSim connection and stop simulation"""
        try:
            # # Stop simulation if it's running
            # sim_state = self.sim.getSimulationState()
            # if sim_state != self.sim.simulation_stopped:
            #     self.get_logger().info('Stopping CoppeliaSim simulation...')
            #     self.sim.stopSimulation()
            #     # Wait a bit for simulation to stop
            #     time.sleep(0.5)
            
            # Close the ZMQ connection
            if hasattr(self, 'client') and self.client:
                self.get_logger().info('Closing CoppeliaSim ZMQ connection...')
                # The client should automatically close when it goes out of scope,
                # but we can explicitly clean up if needed
                self.client = None
                
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {str(e)}')

    def __del__(self):
        """Destructor to ensure cleanup when object is destroyed"""
        self.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = OakSim()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Received Ctrl+C, shutting down gracefully...')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        # Cleanup CoppeliaSim connection before destroying the node
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()