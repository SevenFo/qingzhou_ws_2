#!/usr/bin/env python
# -*- coding: utf-8 -*-
## 添加在车道线入口不停止的参数
import rospy
import numpy
from dynamic_reconfigure import client
from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
import tf
from qingzhou_bringup.srv import app

class ParamCondition():
    def __init__(name = "",xmin = 0,xmax = 0,ymin = 0,ymax = 0,goalstatus = GoalStatus(),params = {}):
        self.name = name
        # self.pose = pose
        self.goalstatus = goalstatus
        self.params = params
        


class DynamicParamsClient():
    def __init__(self):
        rospy.init_node("DynamicParamsClient")
        rospy.loginfo("wait for service")
        #self.log("unloadtorlstart","wait for service")
        rospy.wait_for_service("/move_base/DWAPlannerROS/set_parameters",timeout=100)
        rospy.wait_for_service("/TCP_Sender/app",timeout = 100)
 
        self.debug = True
        self.DWAClient = client.Client("/move_base/DWAPlannerROS",0.2,self.DWAParamsChangedCallback)
        self.goalStatusSuber = rospy.Subscriber("/TCP_Sender/GoalStatus",GoalStatus,self.GoalStatusCallback,queue_size=1)
        self.poseSuber = rospy.Subscriber("/TCP_Sender/PoseInMap",Pose,self.PoseCallback,queue_size=2)
        self.cmdPuber = rospy.Publisher("/cmd_vel",Twist,queue_size=3)
        self.tcpsenderAppClient = rospy.ServiceProxy("/TCP_Sender/app",app)
        self.DWAConfig = None
        self.pose = Pose()
        self.goalStatus = GoalStatus()

        self.openUnloadToRLstart = False
        self.openLoadToUnload = False
        self.DWACondition = [False,False]
        self.openStartToLoad = False
        self.openRLouttostart = False
        self.yaw = None
        self.srv = rospy.Service("/DynamicParamsClient",app,self.srvCallback)
        rospy.loginfo("set up dpc")

    def srvCallback(self,req):
        self.DWACondition[0] = False
        self.DWACondition[1] = False
        if(req.statue == 1):
            self.DWACondition[0] = False
            self.DWACondition[1] = False
            self.log("unloadtorlstart","open unload to rl start")
            self.openUnloadToRLstart = True
        if(req.statue == 2):
            self.openLoadToUnload = True
        if(req.statue == 3):
            self.DWACondition[0] = False
            self.DWACondition[1] = False
            self.log("unloadtorlstart","open start to load")
            self.openStartToLoad = True
        if(req.statue == 4):
            #出S道
            self.DWACondition[0] = False
            self.DWACondition[1] = False
            self.log("unloadtorlstart","open start to load")
            self.openRLouttostart = True
        return 0
    def RLouttostart(self):
        if(not self.DWACondition[0] and not self.DWACondition[1]):#进入装货区前减速
            self.DWAConfig = self.DWAClient.get_configuration()#保存旧的配置
            # self.log("DWAParamCallback","old min_vel_x:{}".format(self.DWAConfig["min_vel_x"]))
            self.DWAClient.update_configuration({"max_vel_x":1.2})
            self.log("unloadtorlstart","set max_vel_x 1.2")
            self.DWACondition[0] = True
        if(self.DWACondition[0] and not self.DWACondition[1] and self.pose.position.x>-0.1): #恢复配置
            self.DWAClient.update_configuration({"max_vel_x":self.DWAConfig["max_vel_x"]})
            self.log("unloadtorlstart","reset max_vel_x:{}".format(self.DWAConfig["max_vel_x"]))
            self.DWAConfig[0] = False
            self.DWAConfig[1] = False
            self.openRLouttostart = False
    def PoseCallback(self,pose):
        self.pose = pose
        (r,p,self.yaw) = tf.transformations.euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
        self.yaw = self.yaw/numpy.pi*180

        # _t = Twist()
        # _t.linear.x = 0.5
        # _t.angular.z = _t.linear.x/1*3*(-90-self.yaw)*numpy.pi/180 + (1.53-self.pose.position.x)*3
        # self.cmdPuber.publish(_t)

        # print(self.yaw)
        # if(self.DWAMinVelXChanged and pose.position.x > -1.2):
        #     self.DWAClient.update_configuration({"min_vel_x":self.DWAConfig["min_vel_x"],"max_vel_theta":self.DWAConfig["max_vel_theta"],"min_vel_theta":self.DWAConfig["min_vel_theta"]})
        #     self.log("DWAParamCallback","min_vel_x set to:".format(self.DWAConfig["min_vel_x"]))
        #     self.DWAMinVelXChanged = False
        pass
    
    def GoalStatusCallback(self,status):
        self.goalStatus = GoalStatus
        # if(status.goal_id.id == "[road line start]" and not self.DWAMinVelXChanged and self.pose.position.x<-2.0):
        #     self.DWAConfig = self.DWAClient.get_configuration()
        #     self.log("DWAParamCallback","old min_vel_x:{}".format(self.DWAConfig["min_vel_x"]))
        #     self.DWAClient.update_configuration({"min_vel_x":0.8,"min_vel_theta":0.1,"max_vel_theta":1.3})
        #     self.DWAMinVelXChanged = True
        # pass
    def loadtounload(self):
        if(not self.DWACondition[0] and not self.DWACondition[1] and self.pose.position.y<-6.0):#过红绿灯减速
            self.DWAConfig = self.DWAClient.get_configuration()#保存旧的配置
            # self.log("DWAParamCallback","old min_vel_x:{}".format(self.DWAConfig["min_vel_x"]))
            self.DWAClient.update_configuration({"max_vel_x":0.8})
            self.log("unloadtorlstart","set max_vel_x 0.8")
            self.DWACondition[0] = True
        if(self.DWACondition[0] and not self.DWACondition[1] and self.pose.position.x<-1.3):#减速带前减速
            self.DWAClient.update_configuration({"max_vel_x":0.5})
            self.log("unloadtorlstart","set max_vel_x 0.5")
            self.DWACondition[0] = True
            self.DWACondition[1] = True
        if(self.DWACondition[0] and self.DWACondition[1] and self.pose.position.y >-6.8): #恢复配置
            self.DWAClient.update_configuration({"max_vel_x":self.DWAConfig["max_vel_x"]})
            self.log("unloadtorlstart","reset max_vel_x:{}".format(self.DWAConfig["max_vel_x"]))
            self.DWAConfig[0] = False
            self.DWAConfig[1] = False
            self.openLoadToUnload = False

    def starttoload(self):
        if(not self.DWACondition[0] and not self.DWACondition[1]):#进入装货区前减速
            self.DWAConfig = self.DWAClient.get_configuration()#保存旧的配置
            # self.log("DWAParamCallback","old min_vel_x:{}".format(self.DWAConfig["min_vel_x"]))
            self.DWAClient.update_configuration({"min_vel_x":1.0})
            self.log("unloadtorlstart","set min_vel_x 1.2")
            self.DWACondition[0] = True
        if(self.DWACondition[0] and not self.DWACondition[1] and self.pose.position.y <-3.5): #恢复配置
            self.DWAClient.update_configuration({"min_vel_x":self.DWAConfig["min_vel_x"]})
            self.log("unloadtorlstart","reset min_vel_x:{}".format(self.DWAConfig["min_vel_x"]))
            self.DWAConfig[0] = False
            self.DWAConfig[1] = False
            self.openStartToLoad = False

    

    def unloadtorlstart(self):
        if(not self.DWACondition[0] and not self.DWACondition[1]):
            # 提高速度
            self.DWAConfig = self.DWAClient.get_configuration()#保存旧的配置
            # self.log("DWAParamCallback","old min_vel_x:{}".format(self.DWAConfig["min_vel_x"]))
            self.DWAClient.update_configuration({"min_vel_x":1.0,"min_vel_theta":0.1,"max_vel_theta":2.0})
            self.log("unloadtorlstart","set min_vel_x 1.0; min_vel_theta 0.1； max_vel_theta: 1.5")
            self.DWACondition[0] = True
        elif(self.DWACondition[0] and not self.DWACondition[1] and self.pose.position.x > -0.2):
            self.DWAClient.update_configuration({"min_vel_x":self.DWAConfig["min_vel_x"],"max_vel_theta":1.3,"min_vel_theta":self.DWAConfig["min_vel_theta"],"max_vel_x":0.5})
            self.log("unloadtorlstart","set min_vel_x {}; min_vel_theta {}； max_vel_theta: {}; max_vel_x:0.5".format(self.DWAConfig["min_vel_x"],self.DWAConfig["min_vel_theta"],1.3))
            # self.log("DWAParamCallback","min_vel_x set to:".format(self.DWAConfig["min_vel_x"]))
            # self.DWACondition[0] = False
            self.DWACondition[1] = True
            self.tcpsenderAppClient(1)#call service to weak thread
        elif(self.DWACondition[1] and self.DWACondition[0] and self.pose.position.y > -4.16):
            self.DWAClient.update_configuration({"max_vel_x":self.DWAConfig["max_vel_x"],"max_vel_theta":self.DWAConfig["max_vel_theta"],"min_vel_x":self.DWAConfig["min_vel_x"]})
            self.log("unloadtorlstart","reset max_vel_x:{},max_vel_theta:{},min_vel_x:{}".format(self.DWAConfig["max_vel_x"],self.DWAConfig["max_vel_theta"],self.DWAConfig["min_vel_x"]))
            self.DWACondition[1] = False
            self.DWACondition[0] = False
            self.openUnloadToRLstart = False

    def DWAParamsChangedCallback(self,config):
        # if(config is None):
        #     self.log("DWAParamsClient","params is none")
        # else:
        #     self.log("DWAParamsClient","params changed")
        pass

    def log(self,name,content):
        if(self.debug):
            rospy.loginfo(content,logger_name=name)

dc = DynamicParamsClient()
r = rospy.Duration(0.03)
while(not rospy.is_shutdown()):
    if(dc.openUnloadToRLstart):
        dc.unloadtorlstart()
    if(dc.openLoadToUnload):
        dc.loadtounload()
    if(dc.openStartToLoad):
        dc.starttoload()
        pass
    if(dc.openRLouttostart):
        dc.RLouttostart()

    rospy.sleep(r)
    