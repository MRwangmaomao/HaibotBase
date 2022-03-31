#!/bin/bash

{
	gnome-terminal -t "haibot_serial" -x bash -c "roslaunch haibot_serial bringup.launch;exec bash"
}&
sleep 1s
{
	gnome-terminal -t "follow_line" -x bash -c "rosrun image_follow_line image_follow_line_node /dev/video0 /dev/video1 0.18 0.005 0.5 30;exec bash"
}  

