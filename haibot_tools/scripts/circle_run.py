#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

INTRODUCTION = """
---------------------------
发布控制命令使小车以圆形轨迹运动
    按下空格后小车停止
---------------------------
"""
# 打印功能信息
print(INTRODUCTION)
settings = termios.tcgetattr(sys.stdin)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# 初始化 ros 节点
rospy.init_node("draw_circle", anonymous=True)

# 初始化控制命令发布者
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# 初始化 Twist 控制消息
twist = Twist()
twist.linear.x = 0.1
twist.angular.z = 0.5

# 初始化 ros主循环
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    # 发布控制命令
    
    cmd_vel_pub.publish(twist)
    rate.sleep()
    key = getKey()
    if key == ' ':
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        cmd_vel_pub.publish(twist)
        break
