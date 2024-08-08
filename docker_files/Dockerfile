# Use the ROS2 Foxy base image
FROM osrf/ros:iron-desktop

# make user ros with uid 1000 and sudo
RUN useradd -m ros && echo "ros:ros" | chpasswd && adduser ros sudo


# Set the ROS_DOMAIN_ID to 11
ENV ROS_DOMAIN_ID=11

# set to fastrtps
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Install sympy using pip
RUN sudo apt-get update && apt-get install -y python3-pip python3-venv

# install talker and listener demo node
RUN sudo apt-get install -y ros-iron-demo-nodes-py

#RUN python3 -m venv /opt/venv

#RUN /opt/venv/bin/pip install sympy pymavlink matplotlib watchdog numpy PyYAML

ENV PATH="/opt/venv/bin:$PATH"

COPY ros_entrypoint.sh /start.sh


# run bash to keep the container running
CMD ["/bin/bash"]