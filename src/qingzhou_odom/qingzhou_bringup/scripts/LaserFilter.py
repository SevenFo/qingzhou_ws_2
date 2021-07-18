#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class LaserFilter():
    def __init__(self):
        self.lserScanSuber = rospy.Subscriber("/scan", LaserScan, self.LaserScanSuberCallback)
        self.delta = 0
        self.range = 0
        self.cmdPuber = rospy.Publisher("/cmd_vel",Twist,queue_size=2)
        self.sleepDua = rospy.Duration(0.05)
        self.linear = 0.8
        self.left = 0
        self.right =0
    def Control(self):
        if(self.range != 0):
            _c = Twist()
            _c.linear.x = self.linear
            # _c.angular.z = abs(self.delta)*self.delta*15
            _c.angular.z =  (self.left-self.range/2)*10
            self.cmdPuber.publish(_c)
            rospy.sleep(self.sleepDua)

        

    def LaserScanSuberCallback(self,data):
        # for item in data.ranges:
        #     print("range:{}".format(item))
        # print("=====================")
        # for item in data.intensities:
        #     print("int:{}".format(item))
        # print("=====================")
        # partRange = data.ranges[0:10]
        # # for r in partRange:

        # #     print(r)
        # # print("===============")
        # print(data.range_min)
        
        #filter
    #     filted = list(filter(self.filter_,data.ranges))
    #     for i in filted:
    #         print(i)
    #     print("============")
    # def filter_(self,data):
    #     if(0.1<data<0.3):
    #         return True
    #     else:
    #         return False
    ####angle
        print("angle min:{};angle max:{};delta_angle:{};size:{}".format(data.angle_min,data.angle_max,data.angle_increment,len(data.ranges)))
        
        left = data.ranges[290*2:310*2]
        right = data.ranges[50*2:70*2]
        print("====left====")
        for i in left:
            print(i)
        print("====right===")
        for i in right:
            print(i)
        print("============")
        
        self.left = left_a = np.mean(left)
        self.right = right_a = np.mean(right)
        self.delta = left_a-right_a
        self.range = left_a+right_a
        
        print("left:{};right:{} delta:{} err:{}; range{};".format(left_a,right_a,left_a-right_a,self.left-self.range/2,self.range/2))
        print("============")
        


    
rospy.init_node("LaserFilter",anonymous=False)

lf = LaserFilter()
try:
    while(not rospy.is_shutdown()):
        lf.Control()
except rospy.ROSInterruptException as e:
    print(e)
