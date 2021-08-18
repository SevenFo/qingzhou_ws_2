#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from qingzhou_bringup.srv import app
import tf


class CmdvelFilter(object):
    def __init__(self):
        self.cmd_suber =  rospy.Subscriber("/cmd_vel", Twist, self.cmdvel_sub_cb)
        self.pose_suber = rospy.Subscriber("/TCP_Sender/PoseInMap", Pose, self.pose_sub_cb)
        self.cmd_puber = rospy.Publisher("/cmd_vel_filted", Twist, queue_size=2)
        self.srv = rospy.Service("/cmdvel_filter_client",app,self.srv_callback)
        
        self.cmd_data = Twist()
        self.pose = Pose()
        self.duration = rospy.Duration(0.025)
        self.is_pub_cmd = False
        self.is_open_cmd = True
        self.is_angular_limit = False
        self.is_not_angular_boost = False
        self.load_angular_boost = False

        self.cmd_data_before = Twist()
        self.cmd_data_to_pub = Twist()


    def srv_callback(self,req):
        if(req.statue == 1):
            #stop pub cmd
            rospy.loginfo("cmd filter: stop pub cmd [{},{}]".format(self.pose.position.x,self.pose.position.y))
            self.is_open_cmd = False
            self.is_angular_limit = False
        elif(req.statue == 2):
            rospy.loginfo("cmd filter: open pub cmd")
            self.is_open_cmd = True
            self.is_angular_limit = False
        elif (req.statue == 3):# 角速度限制
            self.is_angular_limit = True
        elif(req.statue == 4):
            self.is_not_angular_boost =True
        elif(req.statue == 5):
            self.is_not_angular_boost = False
            self.load_angular_boost = False
        elif(req.statue == 6):
            self.load_angular_boost = True
        return 0

    
    def cmdvel_sub_cb(self,data):
        self.cmd_data = data
        if(not self.is_not_angular_boost):
            self.cmd_data.angular.z *= 1.2 
        else:
            self.cmd_data.angular.z *= 1.1
        if(self.load_angular_boost):
            self.cmd_data.angular.z *= 1.0

        # print(self.cmd_data)
        self.is_pub_cmd = True
        pass

    def pose_sub_cb(self,data):
        self.pose = data
        (r,p,self.pose.position.z) = tf.transformations.euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
        # print(self.pose.position.x,self.pose.position.y,self.pose.position.z)
        pass

    def cmd_filter(self):
        if(self.is_pub_cmd and self.is_open_cmd):
            if(self.cmd_data_to_pub.linear.x - self.cmd_data.linear.x > 0.2 or self.cmd_data_to_pub.linear.x - self.cmd_data.linear.x < -0.2):
                #过滤过大的速度变化
                rospy.logwarn("注意速度跳变过大")
                self.cmd_data_to_pub.linear.x = self.cmd_data_to_pub.linear.x + (self.cmd_data.linear.x - self.cmd_data_to_pub.linear.x)*0.15
                rospy.loginfo("{} -> {}".format(self.cmd_data.linear.x,self.cmd_data_to_pub.linear.x))
            else:
                self.cmd_data_to_pub = self.cmd_data
            
            if(self.is_angular_limit and self.cmd_data_to_pub.angular.z < 1.5):
                self.cmd_data_to_pub.angular.z = 1.5
            self.cmd_puber.publish(self.cmd_data_to_pub)
            self.cmd_data_before = self.cmd_data # 记录上一次的速度值
            # self.is_pub_cmd = False
            if(self.cmd_data_to_pub.angular.z - self.cmd_data.angular.z >0.2 or self.cmd_data_to_pub.angular.z - self.cmd_data.angular.z <-0.25):
                rospy.logwarn("注意角速度变化过大")
                self.cmd_data_to_pub.angular.z = self.cmd_data_to_pub.angular.z + (self.cmd_data.angular.z - self.cmd_data_to_pub.angular.z)*0.3
                rospy.loginfo("{} -> {}".format(self.cmd_data.angular.z,self.cmd_data_to_pub.angular.z))
            else:
                self.cmd_data_to_pub = self.cmd_data

        pass

    def run(self):
        rospy.init_node("cmdvel_filter")
        print("started cmd filter node")
        while(not rospy.is_shutdown()):
            self.cmd_filter()
            try:
                rospy.sleep(self.duration)
            except rospy.ROSInterruptException as e:
                print(e)
                break


if __name__ == "__main__":
    my_cmd_filter = CmdvelFilter()
    my_cmd_filter.run()