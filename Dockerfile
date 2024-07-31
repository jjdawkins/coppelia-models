# Use the ROS2 Foxy base image
FROM ros:foxy

# Set the ROS_DOMAIN_ID to 11
ENV ROS_DOMAIN_ID=11

# set to fastrtps
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Install sympy using pip
RUN apt-get update && apt-get install -y python3-pip entr
RUN pip3 install sympy pymavlink matplotlib watchdog

# set work dir to /controller
WORKDIR /app

# copy all files from the current directory to /app

# run ros2 topic list
CMD ["bash"]