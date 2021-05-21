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
#include "fcntl.h"

#define SERVER_IP "127.0.0.1"
#define PORT 6666

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;

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
    float getGoodsPointX;
    float getGoodsPointY;
    float throwGoodsPointX;
    float throwGoodsPointY;
    float userPointX;
    float userPointY;
}rcm;

typedef struct RobotStatusMsg
{
    /* data */
    float bettary;//电池电量
    float linearSpeed;//线速度，x
    float angularSpeed;//角速度
    float locationX;
    float locationY;
    float ekfX;
    float ekfY;
}rsm;
typedef struct testS
{
    int tn;
    float tf;
}ts;
class TCP_Sender
{
private:

    const std::string ipaddr = "127.0.0.1";
    const short port = 6666;

    int shSrv;//SocketHandler
    int shCli;

    std::string recvBuff;
    std::string sendBUff;
    
    ros::NodeHandle nh;
    ros::Subscriber bettarySuber;
    ros::Subscriber speedSuber;//suber to /cmd_vel
    ros::Subscriber locationSuber;
    ros::Subscriber ekfPoseSuber;




public:
    TCP_Sender(const ros::NodeHandle &nodeHandler);
    ~TCP_Sender();
    sockaddr_in addrClient;
    rsm robotStatusMsg;
    ros::Duration sleepDur;

    MoveBaseActionClient * moveBaseActionClientPtr;

    move_base_msgs::MoveBaseGoal startPoint;
    move_base_msgs::MoveBaseGoal getGoodsPoint;
    move_base_msgs::MoveBaseGoal throwGoodsPoint;
    
    
    ros::Publisher cmdvelPuber;
    void SubBettaryInfoCB(const std_msgs::Float32::ConstPtr &msg);
    void SubSpeedCB(const geometry_msgs::Twist::ConstPtr &msg);
    void SubLocCB(const nav_msgs::Odometry::ConstPtr &msg);
    void SubEkfPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

    bool SocketInit();
    bool SendMsg(const void* dataPtr,size_t dataSize);
    bool SendRobotStatusInfo();

    bool ConvertToUnlocked();
    bool RetriveMsg(void * buff, size_t buffSize);

};

#endif