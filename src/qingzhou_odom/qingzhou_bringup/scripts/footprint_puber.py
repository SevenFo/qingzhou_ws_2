#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
# def subscriber_nitity_g():
#     print("*************im subscribed g**************")
# def subscriber_nitity_l():
#     print("*************im subscribed l**************")
if __name__ == "__main__":
    dua = rospy.Duration(2.0)
    while(not rospy.is_shutdown()):
        try:
            global_costmap_hd = rospy.init_node("~")
            foot_print = Polygon()
            foot_print.points = [Point32(0.25,0.20,0),Point32(0.27,0,0),Point32(0.25,-0.20,0),Point32(-0.25,-0.20,0),Point32(-0.25,0.20,0)]
            global_foot_print_puber = rospy.Publisher("/move_base/global_costmap/footprint",Polygon,queue_size=1)
            local_foot_print_puber = rospy.Publisher("/move_base/local_costmap/footprint",Polygon,queue_size=1)
            global_foot_print_puber.publish(foot_print)
            local_foot_print_puber.publish(foot_print)
            rospy.sleep(dua)
        except rospy.ROSInterruptException as e:
            ROS_INFO(e)
            rospy.signal_shutdown("shutdown by keyboard")


