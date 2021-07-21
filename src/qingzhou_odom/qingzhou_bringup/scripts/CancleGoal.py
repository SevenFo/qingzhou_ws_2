#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
# import roslib
# roslib.load_manifest("qingzhou_bringup")
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction
import actionlib

rospy.init_node("cancel_all_goal")
client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
client.wait_for_server()
while(not rospy.is_shutdown()):
    rospy.sleep(0.1)
    client.cancel_all_goals()


