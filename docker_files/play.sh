#!/bin/bash

# play back the bag file
nohup ros2 bag play rosbag2_2024_08_14-13_16_33/ &

sleep 1 &&

ros2 topic list &&

ros2 topic hz /rover/act_vel