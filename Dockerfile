# Use the ROS2 Foxy base image
FROM ros:jazzy

# Set the ROS_DOMAIN_ID to 11
ENV ROS_DOMAIN_ID=11

# set to fastrtps
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Install sympy using pip
RUN apt-get update && apt-get install -y python3-pip python3-venv

# install talker and listener demo node
RUN apt-get install -y ros-jazzy-demo-nodes-py

RUN python3 -m venv /opt/venv

RUN /opt/venv/bin/pip install sympy pymavlink matplotlib watchdog numpy PyYAML

ENV PATH="/opt/venv/bin:$PATH"

# set work dir to /controller
WORKDIR /app

# copy all files from the current directory to /app

# run bash to keep the container running
CMD ["/bin/bash"]