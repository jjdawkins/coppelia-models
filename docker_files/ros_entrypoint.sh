#!/bin/bash
set -e

id -u ros &>/dev/null || adduser --quiet --disabled-password --gecos '' --uid ${UID:=1000} ros

source /opt/ros/humble/setup.bash

su ros -c "source /opt/ros/humble/setup.bash && \
          echo 'USER is $USER' && \
          ros2 topic list"
          

exec "$@"