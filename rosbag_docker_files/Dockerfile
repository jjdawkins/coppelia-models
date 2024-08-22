# Use the ROS2 Foxy base image
FROM ros:humble

# Set the ROS_DOMAIN_ID to 11
ENV ROS_DOMAIN_ID=11

# set to fastrtps
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Install sympy using pip
RUN sudo apt-get update && apt-get install -y ros-humble-rosbag2-py

# install talker and listener demo node
RUN sudo apt-get install -y ros-humble-demo-nodes-py

# install ros2 bag
RUN sudo apt-get install -y ros-humble-ros2bag ros-humble-rosbag2-storage-default-plugins ros-humble-rosbag2-transport

WORKDIR /app

COPY rosbag_command.sh /app/rosbag_command.sh

# run bash to keep the container running
CMD ["/bin/bash"]