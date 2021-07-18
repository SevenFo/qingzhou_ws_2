#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist


class SpeedPlanSub():
    def __init__(self,duaTime,pointx,pointy,speedx,speedz):
        self.dua = rospy.Duration(secs=duaTime)
        self.point = Point(pointx,pointy,0)
        self.speed = Twist()
        self.speed.linear.x = speedx
        self.speed.linear.y = self.speed.linear.z = 0
        self.speed.angular.z = speedz
        self.speed.angular.x = self.speed.angular.y
rospy.init_node("AutoController")
cmdPuber = rospy.Publisher("/cmd_vel",Twist,queue_size=1)

while(not rospy.is_shutdown()):

    SpeedPlan = []

    SpeedPlan.append(SpeedPlanSub(0.8,0,0,0.6,0))
    SpeedPlan.append(SpeedPlanSub(3,0,0,0.8,-1.2))
    # SpeedPlan.append(SpeedPlanSub(2.7,0,0,0.8,1.2))
    SpeedPlan.append(SpeedPlanSub(1,0,0,0.0,0))

    for sps in SpeedPlan:
        cmdPuber.publish(sps.speed)
        rospy.sleep(sps.dua)