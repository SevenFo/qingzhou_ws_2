#ifndef __TCP_SENDER_H__
#define __TCP_SENDER_H__
#include "ros/ros.h"
#include "sys/socket.h"
#include "sys/types.h"
#include <vector>
#include "errno.h"
#include "netinet/in.h"
#include "arpa/inet.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "move_base_msgs/MoveBaseAction.h"
#include <actionlib/client/simple_action_client.h>
#include "tf/tf.h"
#include "fcntl.h"
#include "boost/thread.hpp"
#include "qingzhou_bringup/app.h"
#define SERVER_IP "127.0.0.1"
#define PORT 6666



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;
enum TRAFFICLIGHT{red = 0,green = 1,yellow = 0};
enum GOALSTATE{lost = 0,active = 1,reach = 2 ,aborted = 3};
enum ROBOTLOCATION{load,loadingtotfl,tfltounload,unload,tfl,unloadtoload,start,starttoload,unknow,unloadtoroadline,roadline,unloadtostart,roadlineout};
typedef struct RobotState
{
    GOALSTATE goalstate;
    ROBOTLOCATION robotlocation;
    bool inRoadLine;
    bool roadLineOut;
    float roadLinePianyi;
}robotstate;

typedef struct RobotControlMsg //ros 发送过来的消息结构体
{
    /* data */
    int controlFunctionSelector = 0;
    //bit0 speed control
    //bit1 goal set
    float linearSpeed;//线速度，x
    float angularSpeed;//角速度
    float startPointGoalX;
    float startPointGoalY;
    float startPointGoalZ;
    float getGoodsPointX;
    float getGoodsPointY;
    float getGoodsPointZ;
    float throwGoodsPointX;
    float throwGoodsPointY;
    float throwGoodsPointZ;
    float userPointX;
    float userPointY;
    float userPointZ;
}rcm;

typedef struct RobotStatusMsg
{
    /* data */
    float bettary;//电池电量、
    float linearSpeed;//线速度，x
    float angularSpeed;//角速度
    float locationX;
    float locationY;
    float ekfX;
    float ekfY;
    int trafficLight;//红绿灯 红色0 绿色1 未知-1
    int roadlineStatus;//车道线 还未检测到0 抵达车道线起点1 视觉接手控制2 退出车道线3
}rsm;

class TCP_Sender
{
private:

    const std::string ipaddr = "127.0.0.1";
    const short port = 6666;
    int shSrv;//SocketHandler
    int shCli;

    // std::string recvBuff;//数据接受缓冲区
    // std::string sendBUff;//数据发送缓冲区
    
    ros::NodeHandle nh;
    ros::Subscriber bettarySuber;
    ros::Subscriber speedSuber;//suber to /cmd_vel
    ros::Subscriber locationSuber;
    ros::Subscriber ekfPoseSuber;
    ros::Subscriber trafficLightSuber;
    ros::Subscriber pianyiSuber;
    ros::Duration sleepDur;
    ros::ServiceClient visioncontrolclient;


    robotstate robotState; 
    sockaddr_in addrClient;
    rsm robotStatusMsg;//发送到上位机的机器人状态数据
    bool haveDetectedTfl;
    bool haveDetectRL;//是否检测到车道线 用于在进行过程中更改目标点
    int outcount;//计算偏移量为0的次数
    float pianyibefore;

public:
    TCP_Sender(const ros::NodeHandle &nodeHandler);
    ~TCP_Sender();

    MoveBaseActionClient * moveBaseActionClientPtr;
    ros::Publisher cmdvelPuber;
    move_base_msgs::MoveBaseGoal startPoint;
    move_base_msgs::MoveBaseGoal getGoodsPoint;
    move_base_msgs::MoveBaseGoal throwGoodsPoint;
    move_base_msgs::MoveBaseGoal userPoint;
    move_base_msgs::MoveBaseGoal trafficLightStopLine;
    move_base_msgs::MoveBaseGoal lineStart;

    //**********回调函数***************
    void SubBettaryInfoCB(const std_msgs::Float32::ConstPtr &msg);
    void SubSpeedCB(const geometry_msgs::Twist::ConstPtr &msg);
    void SubLocCB(const nav_msgs::Odometry::ConstPtr &msg);
    void SubEkfPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void SubTrafficLightCB(const geometry_msgs::Vector3::ConstPtr &msg);
    void SubLineCB(const  geometry_msgs::Vector3::ConstPtr &msg);
    void GoalDoneCB(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr &result);
    void GoalActiveCB();//目标点激活的回调函数
    //********************************

    bool SocketInit();//初始化socket
    bool SendMsg(const void* dataPtr,size_t dataSize);//封装
    bool SendRobotStatusInfo();//向上位机发送数据
    void RunGoal();//根据位置和状态选择目标点
    bool ConvertToUnlocked();
    bool RetriveMsg(void * buff, size_t buffSize);
    void roadLineControl(); //车道线控制函数
    void CloseSocket();//关闭socket
    void WaitServices()
    {
        ROS_INFO("wait for movebase action server");
        moveBaseActionClientPtr->waitForServer();//等待服务
    }
    void StopVisonControl();//请求停止视觉控制

    // bool RequestVisionControl(qingzhou_bringup::app::Request &req, qingzhou_bringup::app::Response &res);

};

#endif