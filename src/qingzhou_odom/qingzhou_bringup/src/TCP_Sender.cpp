#include "TCP_Sender.h"

TCP_Sender::TCP_Sender(const ros::NodeHandle &nodeHandler)
{
    nh = nodeHandler;//获取节点句柄
    bettarySuber = nh.subscribe<std_msgs::Float32>("/battery",5,&TCP_Sender::SubBettaryInfoCB,this);//电池电量
    speedSuber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",50,&TCP_Sender::SubSpeedCB,this);//速度
    locationSuber = nh.subscribe<nav_msgs::Odometry>("/odom",50,&TCP_Sender::SubLocCB,this);//odom
    ekfPoseSuber = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose_ekf/odom_combined",50,&TCP_Sender::SubEkfPoseCB,this);//ekf
    trafficLightSuber = nh.subscribe<std_msgs::Float32>("/color",5,&TCP_Sender::SubTrafficLightCB,this);;//红绿灯
    robotState.goalstate = lost;
    robotState.robotlocation = start;
    haveDetectedTfl = false;
    sleepDur = ros::Duration(1);
    std::cout<<"creatededed"<<std::endl;
    cmdvelPuber = nh.advertise<geometry_msgs::Twist>("/cmd_vel",50);

    moveBaseActionClientPtr = new MoveBaseActionClient("move_base");
    ROS_INFO("wait for movebase action server");
    moveBaseActionClientPtr->waitForServer();//等待服务
    

    //初始化停止线的位置
    trafficLightStopLine.target_pose.header.frame_id = "map";
    trafficLightStopLine.target_pose.pose.position.x = 2.289;
    trafficLightStopLine.target_pose.pose.position.y = -6.894;
    trafficLightStopLine.target_pose.pose.orientation.x = 0.000;
    trafficLightStopLine.target_pose.pose.orientation.y = 0.000;
    trafficLightStopLine.target_pose.pose.orientation.z = -0.7;
    trafficLightStopLine.target_pose.pose.orientation.w = 0.714;
    
    
    
    
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


//目标点完成的回调函数用于更改robotState.goalstate
void TCP_Sender::GoalDoneCB(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    std::cout<<"i am in call back funciton"<<std::endl;
    //ROS_INFO("Answer: %i", result->sequence.back());
    if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        haveDetectedTfl = false;
        robotState.goalstate = reach;
        switch (robotState.robotlocation)
        {
        case starttoload:
        {
            robotState.robotlocation = load;
            break;
        }
        case tfltounload:
        {
            robotState.robotlocation = unload;
            break;
        }
        case unloadtoload:
        {
            robotState.robotlocation = load;
            break;
        }
        case loadingtotfl:
        {
            robotState.robotlocation = tfl;
        
            break;
        }
        default:
        {
            ROS_WARN("cant get positonstate");
            break;
        }
        }
    }
    else if(state == actionlib::SimpleClientGoalState::ABORTED || state == actionlib::SimpleClientGoalState::PREEMPTED)
    {
        robotState.goalstate = aborted;
    }
    std::cout<<"goal state change to "<<robotState.goalstate << " and now location is "<<robotState.robotlocation<<std::endl;
    
}
void TCP_Sender::GoalActiveCB()
{
    std::cout<<"i am in goal active callback function"<<std::endl;
}

void TCP_Sender::SubTrafficLightCB(const std_msgs::Float32ConstPtr &msg)
{
    robotStatusMsg.trafficLight = (int)(msg.get()->data);
    if(!haveDetectedTfl)
    {
        if((int)(msg.get()->data) == red && robotState.robotlocation != tfl)
        {
            ROS_INFO_STREAM("red");
            haveDetectedTfl = true;//保证只能检测到一次红绿灯，防止频繁更改目标点，当到达下一个目标点的时候才释放红绿灯检测权
            //change goal to stop line
            //boost::thread actionGoalThread(this->moveBaseActionClientPtr->sendGoal,this->moveBaseActionClientPtr,)
            
            this->moveBaseActionClientPtr->sendGoal(trafficLightStopLine, boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            robotState.goalstate = active;
            robotState.robotlocation = loadingtotfl;
            ROS_INFO("going to tfl");
            ROS_INFO_STREAM("GOAL: x:"<<trafficLightStopLine.target_pose.pose.position.x<<" y:"<<trafficLightStopLine.target_pose.pose.position.y<<" z:"<<trafficLightStopLine.target_pose.pose.orientation.z);
        }
        if((int)(msg.get()->data) == green)
        {
            ROS_INFO_STREAM("green");
            //change goal to next postioin
            haveDetectedTfl = true;
            this->moveBaseActionClientPtr->sendGoal(throwGoodsPoint, boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            robotState.goalstate = active;
            robotState.robotlocation = tfltounload;
            ROS_INFO("going to unload");
            ROS_INFO_STREAM("GOAL: x:"<<throwGoodsPoint.target_pose.pose.position.x<<" y:"<<throwGoodsPoint.target_pose.pose.position.y<<" z:"<<throwGoodsPoint.target_pose.pose.orientation.z);
        }
    }

}


void TCP_Sender::RunGoal()
{
    if(robotState.goalstate == reach)//成功到达某个目标点
    {
        switch (robotState.robotlocation)
        {
        case start:
        {
            moveBaseActionClientPtr->sendGoal(getGoodsPoint,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            robotState.robotlocation = starttoload;
            robotState.goalstate = active;
            break;
        }
        case load:
        {
            moveBaseActionClientPtr->sendGoal(trafficLightStopLine,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            robotState.robotlocation = loadingtotfl;
            robotState.goalstate = active;
            ROS_INFO("going to tfl");
            ROS_INFO_STREAM("GOAL: x:"<<trafficLightStopLine.target_pose.pose.position.x<<" y:"<<trafficLightStopLine.target_pose.pose.position.y<<" z:"<<trafficLightStopLine.target_pose.pose.orientation.z);
            break;
        }
        case unload:
        {
            moveBaseActionClientPtr->sendGoal(getGoodsPoint,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            robotState.robotlocation = unloadtoload;
            robotState.goalstate = active;
            ROS_INFO("going to load");
            ROS_INFO_STREAM("GOAL: x:"<<getGoodsPoint.target_pose.pose.position.x<<" y:"<<getGoodsPoint.target_pose.pose.position.y<<" z:"<<getGoodsPoint.target_pose.pose.orientation.z);
            break;
        }
        
        default:
        {
            break;
        }
        }
    }
    else if(robotState.goalstate == aborted) //如果目标在中途异常停止
    {
        switch (robotState.robotlocation)
        {
        case starttoload:
        {
            moveBaseActionClientPtr->sendGoal(getGoodsPoint,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            robotState.robotlocation = starttoload;
            robotState.goalstate = active;
            ROS_INFO("going to load");
            ROS_INFO_STREAM("GOAL: x:"<<getGoodsPoint.target_pose.pose.position.x<<" y:"<<getGoodsPoint.target_pose.pose.position.y<<" z:"<<getGoodsPoint.target_pose.pose.orientation.z);
            break;
        }
        case tfltounload:
        {
            moveBaseActionClientPtr->sendGoal(throwGoodsPoint,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            robotState.robotlocation = tfltounload;
            robotState.goalstate = active;
            ROS_INFO("going to unload");
            ROS_INFO_STREAM("GOAL: x:"<<throwGoodsPoint.target_pose.pose.position.x<<" y:"<<throwGoodsPoint.target_pose.pose.position.y<<" z:"<<throwGoodsPoint.target_pose.pose.orientation.z);
            break;
        }
        case unloadtoload:
        {
            moveBaseActionClientPtr->sendGoal(getGoodsPoint,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            robotState.robotlocation = unloadtoload;
            robotState.goalstate = active;
            ROS_INFO("going to load");
            ROS_INFO_STREAM("GOAL: x:"<<getGoodsPoint.target_pose.pose.position.x<<" y:"<<getGoodsPoint.target_pose.pose.position.y<<" z:"<<getGoodsPoint.target_pose.pose.orientation.z);

            break;
        }
        case loadingtotfl:
        {
            moveBaseActionClientPtr->sendGoal(trafficLightStopLine,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            robotState.robotlocation = loadingtotfl;
            robotState.goalstate = active;
            ROS_INFO("going to tfl");
            ROS_INFO_STREAM("GOAL: x:"<<trafficLightStopLine.target_pose.pose.position.x<<" y:"<<trafficLightStopLine.target_pose.pose.position.y<<" z:"<<trafficLightStopLine.target_pose.pose.orientation.z);
            break;
        }
        default:
        {
            break;
        }
        }
    }
    else if(robotState.goalstate == lost)
    {
        moveBaseActionClientPtr->sendGoal(getGoodsPoint,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2),boost::bind(&TCP_Sender::GoalActiveCB,this));
        robotState.goalstate = active;
        robotState.robotlocation = starttoload;
        ROS_INFO("going to load");
        ROS_INFO_STREAM("GOAL: x:"<<getGoodsPoint.target_pose.pose.position.x<<" y:"<<getGoodsPoint.target_pose.pose.position.y<<" z:"<<getGoodsPoint.target_pose.pose.orientation.z);
    }
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
        //std::cout<<"error: socket send msg failed"<<std::endl;
        return false;
    }
    else
    {
        //std::cout<<"log: socket send msg successed"<<std::endl;
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