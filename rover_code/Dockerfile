# Use the ROS2 Foxy base image
FROM ros:humble

# Set the ROS_DOMAIN_ID to 11
ENV ROS_DOMAIN_ID=11

# set to fastrtps
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Install Rosbag2 
RUN sudo apt-get update && apt-get install -y ros-humble-rosbag2-py

# install talker and listener demo node
RUN sudo apt-get install -y ros-humble-demo-nodes-py

# install ros2 bag
RUN sudo apt-get install -y ros-humble-ros2bag ros-humble-rosbag2-storage-default-plugins ros-humble-rosbag2-transport

# install sympy, numpy, and matplotlib pymavlink and pyserial
RUN sudo apt-get install -y python3-pip
RUN pip3 install sympy numpy matplotlib pymavlink pyserial watchdog

WORKDIR /app

# run bash to keep the container running
CMD ["/bin/bash"]