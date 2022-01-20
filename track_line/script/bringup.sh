#!/bin/bash

sudo chmod 777 /dev/ttyACM0
expect "password"
send "123456\r"

{
	gnome-terminal -t "ros" -x bash -c "roscore;exec bash"
}&
sleep 5s
{
	gnome-terminal -t "pudu_serial" -x bash -c "rosrun pudu_serial serial_write;exec bash"
}&
sleep 1s
{
	gnome-terminal -t "follow_line" -x bash -c "rosrun image_follow_line image_follow_line_node /dev/video0 /dev/video1 0.18 0.005 0.5 30;exec bash"
}  

