#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import time

def srv_callback(req):
    cmdpub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
    clear_map_srv_Client = rospy.ServiceProxy("/move_base/clear_costmaps",Empty)
    global_localization_srv_client = rospy.ServiceProxy("/global_localization",Empty)
    print("**********Waiting for services*************")
    clear_map_srv_Client.wait_for_service()
    global_localization_srv_client.wait_for_service()
    rospy.timer.sleep(rospy.Duration(1))#unkown reason
    # print("Ready and call localizaiton")
    # global_localization_srv_client.call()
    cmd_data = Twist()
    cmd_data.linear.x = -0.3
    print("Robot will move backward for 2s")
    cmdpub.publish(cmd_data)
    rospy.timer.sleep(rospy.Duration(2))
    print("Robot will move forward for 2s")
    cmd_data.linear.x = 0.3
    cmdpub.publish(cmd_data)
    rospy.timer.sleep(rospy.Duration(2))
    cmd_data.linear.x = 0
    cmdpub.publish(cmd_data)
    rospy.timer.sleep(rospy.Duration(1))
    print("Robot localized and trying to clear costmap")
    clear_map_srv_Client.call()
    print("Done! wati 0.5s for clear costmap")
    # rospy.signal_shutdown("over")
    rospy.timer.sleep(rospy.Duration(0.5))
    # return True
    return []

rospy.init_node("firststated")

if __name__ == "__main__":
    # time.sleep(5)
    srv = rospy.Service("/clear_cost_map",Empty,srv_callback)
    
    rospy.spin()
    






