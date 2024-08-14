#!/bin/bash

# Set the output file name and path
description="run_0_current"
output_file=f"rosbags/$description.bag"

# Start recording the /rover/cmd_vel topic
rosbag record -O "$output_file" /rover/cmd_vel /rover/act_vel /rover/ref_vel /rover/est_param