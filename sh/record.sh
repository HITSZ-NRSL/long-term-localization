#!/usr/bin/zsh

cd ~/offline_process/sequences/00/
rosbag record /lio_sam/mapping/odometry /navgps /navodom /imu /odom /clock -O semantic.bag