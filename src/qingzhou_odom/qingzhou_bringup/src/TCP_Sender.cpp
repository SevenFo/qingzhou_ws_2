#include "TCP_Sender.h"

TCP_Sender::TCP_Sender(const ros::NodeHandle &nodeHandler)
{
    nh = nodeHandler;//获取节点句柄
    bettarySuber = nh.subscribe<std_msgs::Float32>("/battery",5,&TCP_Sender::SubBettaryInfoCB,this);
    speedSuber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",50,&TCP_Sender::SubSpeedCB,this);
    locationSuber = nh.subscribe<nav_msgs::Odometry>("/odom",50,&TCP_Sender::SubLocCB,this);
    ekfPoseSuber = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose_ekf/odom_combined",50,&TCP_Sender::SubEkfPoseCB,this);
    sleepDur = ros::Duration(1);
    std::cout<<"creatededed"<<std::endl;
    cmdvelPuber = nh.advertise<geometry_msgs::Twist>("/cmd_vel",50);

    moveBaseActionClientPtr = new MoveBaseActionClient("move_base");
    ROS_INFO("wait for movebase action server");
    moveBaseActionClientPtr->waitForServer();//等待服务

}

TCP_Sender::~TCP_Sender()
{
    delete moveBaseActionClientPtr;
}


void TCP_Sender::SubBettaryInfoCB(const std_msgs::Float32::ConstPtr &msg)
{
    robotStatusMsg.bettary = msg.get()->data;
   // std::cout<<"log :set bettary value"<<std::endl;

}
void TCP_Sender::SubSpeedCB(const geometry_msgs::Twist::ConstPtr &msg)
{
    robotStatusMsg.linearSpeed =msg.get()->linear.x;
    robotStatusMsg.angularSpeed=msg.get()->angular.z;
}
void TCP_Sender::SubLocCB(const nav_msgs::Odometry::ConstPtr &msg)
{
    robotStatusMsg.locationX = msg.get()->pose.pose.position.x;
    robotStatusMsg.locationY = msg.get()->pose.pose.position.y;
    //方向todo
}
void TCP_Sender::SubEkfPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{

    robotStatusMsg.ekfX = msg.get()->pose.pose.position.x;
    robotStatusMsg.ekfY = msg.get()->pose.pose.position.y;
}
bool TCP_Sender::SocketInit()//初始化SOCKET
{
    shSrv = socket(AF_INET,SOCK_STREAM,0);//TCP STREAM
    if(shSrv == -1)
    {
        std::cout<<"error: socket init failed"<<std::endl;
        return false;
    }
    else
    {
        std::cout<<"log: socket inited"<<std::endl;
    }

    sockaddr_in addrSrv;

    addrSrv.sin_family = AF_INET;

    addrSrv.sin_port = htons(PORT);
   
    addrSrv.sin_addr.s_addr = INADDR_ANY;

    
    if(bind(shSrv,(sockaddr*)&addrSrv,sizeof(sockaddr)))
    {
        std::cout<<"error: socket bind failed"<<std::endl;
        return false;
    }
    else
    {
        //
    }
    std::cout<<"log: socket listening port 6666..."<<std::endl;
    if(listen(shSrv,5))
    {
        std::cout<<"error: socket listen failed"<<std::endl;
        return false;
    }

    size_t sockaddrSize = sizeof(sockaddr_in);
    shCli = accept(shSrv,(sockaddr*)&addrClient,(socklen_t*)&sockaddrSize);
    if(-1 == shCli)
    {
        std::cout<<"error: socket accept failed"<<std::endl;
        return false;
    }
    std::cout<<"log: connected to client"<<std::endl; 
    if(ConvertToUnlocked())//转化成非阻塞
        return true;
    else
        return false;
}

bool TCP_Sender::SendMsg(const void* dataPtr,size_t dataSize)
{
    if(send(shCli,dataPtr,dataSize,0) == -1)
    {
        std::cout<<"error: socket send msg failed"<<std::endl;
        return false;
    }
    else
    {
        std::cout<<"log: socket send msg successed"<<std::endl;
        return true;
    }
}

bool TCP_Sender::SendRobotStatusInfo()//发送机器人当前的状态
{
    return SendMsg(&robotStatusMsg,sizeof(robotStatusMsg));
}



bool TCP_Sender::ConvertToUnlocked()
{
    if(fcntl(shCli, F_SETFL, fcntl(shCli, F_GETFL, 0) | O_NONBLOCK) == -1)
    {
        std::cout<<"error: convert to unblocked failed"<<std::endl;
        return false;
    }
    else
    {
        return true;
    }
}

bool TCP_Sender::RetriveMsg(void *buff,size_t buffSize)//从sicket取回信息
{
    auto err = recv(shCli,buff,buffSize,0);
    if(err == -1 || err == 0)
    {
        //std::cout<<"error: none msg to retrive"<<std::endl;
        return false;
    }
    else
    {
        std::cout<<"log: retrive "<<err<<" bytes"<<std::endl;
        return true;
    }
}