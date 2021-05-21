#!/usr/bin/env python
"""
transmit msg from /key_Vel to /cmd_Vel
"""
import rospy
from geometry_msgs.msg import Twist

class TeleTransmitor():
    def __init__(self):
        rospy.init_node("tele_msg_transmitor_node",anonymous=False)

        self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=50)
        rospy.loginfo("wait for msg key_vel")
        rospy.wait_for_message("/key_vel",Twist)

        rospy.Subscriber("/key_vel",Twist,self.Trans)

        rospy.loginfo("INIT MSG TRANSMITOR SUCCESS")

    def Trans(self, msg):
        self.pub.publish(msg)
    
if __name__ == "__main__":
    try:
        myTransmitor = TeleTransmitor()
        rospy.spin()
    except Exception as e:
        rospy.logerr(e)
    