#!/bin/bash
echo "This script can cartographer and navigation!"

echo "Update env!"
source /opt/ros/melodic/setup.bash
source /home/robot/rgbd_catkin/devel/setup.bash
source /home/robot/catkin_ws/devel/setup.bash

roslaunch haibot_tools haibot_bringup.launch &

sleep 5
echo "Start haibot robot cartographer slam."
roslaunch cartographer_slam_core haibot_cartographer.launch &

sleep 2
echo "Start haibot robot global and local navigation."
roslaunch nav move_base.launch &

sleep 2
echo "rviz visaulizaiton."
roslaunch cartographer_slam_core visualization.launch &
