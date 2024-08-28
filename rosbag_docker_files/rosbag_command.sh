#!/bin/bash

# Start recording the /rover/cmd_vel topic
ros2 bag record /rover/cmd_vel /rover/act_vel /rover/ref_vel /rover/est_param /rover/mocap/odom /qualisys/rover/pose



