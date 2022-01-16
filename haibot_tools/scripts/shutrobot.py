#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from haibot_serial.msg import Shutdown
import os
import time

msg = """
-----------------------------------
          Shutdown Haibot !
               Bye !!!
-----------------------------------
"""

if __name__=="__main__":
    rospy.init_node('shut_robot')
    pub = rospy.Publisher('/shutdown', Shutdown, queue_size=3)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        try:
            print(msg)
            shut_robot = Shutdown()
            shut_robot.delay_time = 20 # 20 seconds robot mcu power down.
            pub.publish(shut_robot)
            rate.sleep()
        except rospy.ROSInterruptException as e:
            print(e)
        finally:
            time.sleep(3)
            os.system('shutdown -h now')
