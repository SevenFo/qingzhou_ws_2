#include "TCP_Sender.h"
#include "math.h"
TCP_Sender::TCP_Sender(const ros::NodeHandle &nodeHandler){
    nh = nodeHandler;//获取节点句柄
    bettarySuber = nh.subscribe<std_msgs::Float32>("/battery",5,&TCP_Sender::SubBettaryInfoCB,this);//电池电量
    speedSuber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",50,&TCP_Sender::SubSpeedCB,this);//速度
    locationSuber = nh.subscribe<nav_msgs::Odometry>("/odom",50,&TCP_Sender::SubLocCB,this);//odom
    locationInMapSuber = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",50,&TCP_Sender::SubLcationMapCB,this);//odom
    ekfPoseSuber = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose_ekf/odom_combined",50,&TCP_Sender::SubEkfPoseCB,this);//ekf
    trafficLightSuber = nh.subscribe<geometry_msgs::Vector3>("/pianyi",5,&TCP_Sender::SubTrafficLightCB,this);//红绿灯
    pianyiSuber = nh.subscribe<geometry_msgs::Vector3>("/pianyi",1,&TCP_Sender::SubLineCB,this);
    visioncontrolclient = nh.serviceClient<qingzhou_bringup::app>("/vision_control");
    clearCostmapFirstlyClient = nh.serviceClient<std_srvs::Empty>("/clear_cost_map");
    clearCostmapClient = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

    robotState.goalstate = lost;
    robotState.robotlocation = start;
    robotState.inRoadLine = false;
    robotState.autoGoalControl = false;//比赛规则不允许自动发布目标点
    robotState.openTflDet = false;
    // robotState.roadLineOut = false;
    pianyibefore = 1;
    outcount = 0;
    haveDetectedTfl = false;
    sleepDur = ros::Duration(1);
    std::cout<<"creatededed"<<std::endl;
    cmdvelPuber = nh.advertise<geometry_msgs::Twist>("/cmd_vel",50);
    // roadlineControlPuber = nh.advertise<std_msgs::Float32>("/is_version_cont",10);

    moveBaseActionClientPtr = new MoveBaseActionClient("move_base");
    ROS_INFO_NAMED("TCP_Sender", "Waiting services");
    clearCostmapClient.waitForExistence();

    

    //初始化停止线的位置
    trafficLightStopLine.target_pose.header.frame_id = "map";
    trafficLightStopLine.target_pose.pose.position.x = 2.260;
    trafficLightStopLine.target_pose.pose.position.y = -6.291;
    trafficLightStopLine.target_pose.pose.orientation.x = 0.000;
    trafficLightStopLine.target_pose.pose.orientation.y = 0.000;
    trafficLightStopLine.target_pose.pose.orientation.z = -0.691;
    trafficLightStopLine.target_pose.pose.orientation.w = 0.723;

    lineStart.target_pose.header.frame_id = "map";
    lineStart.target_pose.pose.position.x = 1.026;
    lineStart.target_pose.pose.position.y = -3.7500;
    lineStart.target_pose.pose.orientation.x = 0.000;
    lineStart.target_pose.pose.orientation.y = 0.000;
    lineStart.target_pose.pose.orientation.z = 0.6862315947380534;
    lineStart.target_pose.pose.orientation.w = 0.7273831166471133;
    
    
    
    
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
void TCP_Sender::SubLcationMapCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    robotStatusMsg.locationInMapY = msg.get()->pose.pose.position.y;
    robotStatusMsg.locationInMapX = msg.get()->pose.pose.position.x;
}
void TCP_Sender::SubTrafficLightCB(const geometry_msgs::Vector3::ConstPtr &msg)
{
    robotStatusMsg.trafficLight = (int)(msg.get()->x);
    // ROS_INFO_STREAM("tfl data: "<<robotStatusMsg.trafficLight);
    if(!haveDetectedTfl)//如果还没还没有检测到交通灯，那就执行下面的程序，检测话题值
    {

        if(robotStatusMsg.trafficLight == 666 && robotState.robotlocation != tfl)//如果是红灯，而且机器人当前的位置不在停止线那就前往停止线等待绿灯
        {
            ROS_INFO_STREAM("red");
            haveDetectedTfl = true;//保证只能检测到一次红绿灯，防止频繁更改目标点，当到达下一个目标点的时候才释放红绿灯检测权
            //change goal to stop line
            //boost::thread actionGoalThread(this->moveBaseActionClientPtr->sendGoal,this->moveBaseActionClientPtr,)
            

            //发送前往停止线的命令
            this->moveBaseActionClientPtr->sendGoal(trafficLightStopLine, boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            robotState.goalstate = active;
            robotState.robotlocation = loadingtotfl;
            ROS_INFO("going to tfl");
            ROS_INFO_STREAM("GOAL: x:"<<trafficLightStopLine.target_pose.pose.position.x<<" y:"<<trafficLightStopLine.target_pose.pose.position.y<<" z:"<<trafficLightStopLine.target_pose.pose.orientation.z);
        }

        if(robotStatusMsg.trafficLight == 777 ||robotStatusMsg.trafficLight == 888 )//如果是绿灯，不管车在哪里都直接前往下一个目标点
        {
            ROS_INFO_STREAM("green");
            qingzhou_bringup::app req;
            req.request.statue = 4;
            if (visioncontrolclient.call(req))
            {
                ROS_INFO_NAMED("TCP_Sender", "closed color detector");
            }
            else
            {
                //todo
            }
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


void TCP_Sender::SubLineCB(const geometry_msgs::Vector3::ConstPtr &msg)
{
    int pianyi = (msg.get()->y);
    if(robotState.inRoadLine)//不过，在开启 vision的request还没发送给脚本的话，该话题是没有pianyi可以读取的，所以这个条件应该可以删了
    {
        if(int(pianyi))//pianyi >0 才进入
        {
            robotState.roadLinePianyi = pianyi;
            ROS_INFO_STREAM("pianyi "<<pianyi<<" devid :"<<abs(pianyi - pianyibefore));
            if(pianyi >998 ){//偏移跳变大于30判断退出赛道 不太好用 废弃了
                //如果车在车道线里面，并且检测不到车道线，才判断location在roadlineout
                if(robotState.robotlocation == roadline)
                {
                    StopVisonControl();
                }
                
            }
        }
        else{
            ROS_INFO_STREAM("pianyi " << pianyi);
        }
    }
    pianyibefore = pianyi;
}


//目标点完成的回调函数用于更改robotState.goalstate
void TCP_Sender::GoalDoneCB(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());

    std::cout << "i am a goal done cb" << std::endl;

    std_srvs::Empty req;
    if (clearCostmapClient.call(req)) {
        ROS_INFO_NAMED("TCP_Sender", "clear cost map success!");
        //在转角的时候路径规划没有考虑障碍物？
        ROS_INFO_NAMED("TCP_Sender", "Sleep for 0.8 second,(wait costmap to update)");
        sleepDur = ros::Duration(0.8);
        sleepDur.sleep();
        ROS_INFO_NAMED("TCP_Sender", "Weak up(wait costmap to update)");


    }
    else
    {
        ROS_INFO_NAMED("TCP_Sender", "clear cost map failed!");
    }

    //ROS_INFO("Answer: %i", result->sequence.back());
    //出发点-转载点 -> 装载点； 装载点 - 交通灯->交通灯； 交通灯 - 卸货点->卸货点； 卸货点 - 车道线出发点->车道线出发点 ->
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        
        haveDetectedTfl = false;
        robotState.goalstate = reach;
        switch (robotState.robotlocation)
        {
        case starttoload: {
            robotState.robotlocation = load;
            break;
            }
        case tfltounload:{robotState.robotlocation = unload;break;}
        //废弃
        case unloadtoload:{robotState.robotlocation = load;break;}
        //当前使用
        case unloadtostart:{robotState.robotlocation = start;break;}
        case loadingtotfl:{robotState.robotlocation = tfl;break;}
        case unloadtoroadline:
        {
            robotStatusMsg.roadlineStatus = 1;
            //更改一下从unloadtoroadline切换到roadline的条件，感觉和RobotState.inRoadLine这个条件变量冲突了
            //不冲突，RobotState.inRoadLine 保证当视觉开启之后就不再请求开启视觉控制的service，之后当service请求成功之后set true，否则会一直尝试
            ROS_INFO("change unloadtoroadline to roadline and inroadline set true");
            robotState.robotlocation = roadline;
            qingzhou_bringup::app req;
            req.request.statue = 1;
            if (visioncontrolclient.call(req))
            {
                robotState.inRoadLine=true;//神经网络接手控制
                robotStatusMsg.roadlineStatus = 2;
                ROS_INFO_NAMED("TCP_Sender", "open roadline detector succrss");
            }
            else
            {
                ROS_INFO_NAMED("TCP_Sender", "open roadline detector failed， robot well try to call it later");
                //todo
            }
            
            break;
        }
        default:{ROS_WARN("cant get positonstate");break;}
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


void TCP_Sender::RunGoal()
{
    robotStatusMsg.robotstateMsg = this->robotState;//更新robotstate到上位机
    //如果成功到达某个目标点那就前往下一个目标点
    //出发点 -> 装载点 -> 交通灯停止线 -> （由话题回调函数控制，在交通灯停止线等待绿灯） -> 卸载区 ->
    //(车道线 -> 发送请求视觉控制 ->（若请求成功则开始检测是否退出车道线） -> 车道线外面: 这个过程中状态都是reach) -> 出发点
    // ROS_INFO_STREAM("run goal state:"<<robotState.goalstate);
    if(robotState.goalstate == reach)//成功到达某个目标点
    {
        // ROS_INFO_STREAM("reach:");
        robotState.goalstate = active;//reach状态只能进入一次（每次goalreach）
        switch (robotState.robotlocation)
        {
        case start:
        {
            //前往getGood point
            moveBaseActionClientPtr->sendGoal(getGoodsPoint,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            robotState.robotlocation = starttoload;
            robotState.goalstate = active;
            break;
        }
        case load:
        {
            if(robotState.autoGoalControl)//只有开启autogoalcontrol之后才可以一到达装货区就发布去停止线的goal，否则要我们手动发布
            {
                moveBaseActionClientPtr->sendGoal(trafficLightStopLine,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
                robotState.robotlocation = loadingtotfl;
                robotState.goalstate = active;
                ROS_INFO("going to tfl");
                ROS_INFO_STREAM("GOAL: x:"<<trafficLightStopLine.target_pose.pose.position.x<<" y:"<<trafficLightStopLine.target_pose.pose.position.y<<" z:"<<trafficLightStopLine.target_pose.pose.orientation.z);
            }
            else{
                ROS_INFO_NAMED("TCP_Sender", "robot have reach load, pls pub goal to traffic light stop line");
            }
            break;
        }
        case tfl:
        {
            //到达tfl之后如果没有开启红路灯探测，就开启红绿灯的探测，可能会有一点延迟，后面要改进一点
            if(!robotState.openTflDet)
            {
                qingzhou_bringup::app req;
                req.request.statue = 3;
                if (visioncontrolclient.call(req))
                {
                    robotState.openTflDet = true;
                    ROS_INFO_NAMED("RCP_Sender", "open color detector success");
                }
                else
                {
                    //todo
                    ROS_INFO_NAMED("RCP_Sender", "open color detector failed");

                }
            }
            else{
                 ROS_INFO_NAMED("RCP_Sender", "robot have opened color detector, detecting tlf...");
            }
            break;
        }
        case unload:
        {
            // moveBaseActionClientPtr->sendGoal(startPoint,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            // robotState.robotlocation = unloadtostart;
            // robotState.goalstate = active;
            // ROS_INFO("going to start");
            // ROS_INFO_STREAM("GOAL: x:"<<startPoint.target_pose.pose.position.x<<" y:"<<startPoint.target_pose.pose.position.y<<" z:"<<startPoint.target_pose.pose.orientation.z);
            if(robotState.autoGoalControl)//只有开启autogoalcontrol之后才可以一到卸货区就发布去roalline的goal，否则要我们手动发布
            {
                moveBaseActionClientPtr->sendGoal(lineStart,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
                robotState.robotlocation = unloadtoroadline;
                robotState.goalstate = active;
                ROS_INFO("going to roadline");
                ROS_INFO_STREAM("GOAL: x:"<<startPoint.target_pose.pose.position.x<<" y:"<<startPoint.target_pose.pose.position.y<<" z:"<<startPoint.target_pose.pose.orientation.z);
            }
            else{
                ROS_INFO_NAMED("TCP_Sender", "robot have reached unload, pls pub the goal to roadline/start area");
            }
            break;
        }
        case roadlineout:
        {
            ROS_INFO_STREAM("road line out and  to start and location is "<<robotState.robotlocation);
            //退出车道线之后重新回到unloadtostart的目标轨迹中
            moveBaseActionClientPtr->sendGoal(startPoint,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            robotState.robotlocation = unloadtostart;
            robotState.goalstate = active;
            ROS_INFO("going to start");
            ROS_INFO_STREAM("GOAL: x:"<<startPoint.target_pose.pose.position.x<<" y:"<<startPoint.target_pose.pose.position.y<<" z:"<<startPoint.target_pose.pose.orientation.z);
            break;
        }
        case roadline:
        {
            //视觉接管控制
            // ROS_INFO("in road line");
            //在前面的 call back 已经请求过一次service，若果失败了这边才会再次请求
            if(!robotState.inRoadLine){
                
                // roadLineControl();

                // std_msgs::Float32 data;
                // data.data = 1;
                // roadlineControlPuber.publish(data);
                qingzhou_bringup::app req;
                req.request.statue = 1;
                if(visioncontrolclient.call(req))
                {
                    robotState.inRoadLine=true;//神经网络接手控制
                    robotStatusMsg.roadlineStatus = 2;
                    ROS_INFO("call vison control service success!");
                }
                else
                {
                    ROS_INFO("call vison control service failed!，robot will try to call this service again");
                }

            }
            break;
        }
        default:
        {
            ROS_ERROR_NAMED("TCP_Sender", "something worong happened, robot have reach an unknown area!");
            break;
        }
        }
    }
    else if(robotState.goalstate == aborted) //如果目标在中途异常停止
    {
        // ROS_INFO_STREAM("abort:");
        switch (robotState.robotlocation)
        {
        case starttoload:
        {
            moveBaseActionClientPtr->sendGoal(getGoodsPoint,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            robotState.robotlocation = starttoload;
            robotState.goalstate = active;
            ROS_INFO("robot can't reach the goal, try to go load again");
            ROS_INFO_STREAM("GOAL: x:"<<getGoodsPoint.target_pose.pose.position.x<<" y:"<<getGoodsPoint.target_pose.pose.position.y<<" z:"<<getGoodsPoint.target_pose.pose.orientation.z);
            break;
        }
        case tfltounload:
        {
            moveBaseActionClientPtr->sendGoal(throwGoodsPoint,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            robotState.robotlocation = tfltounload;
            robotState.goalstate = active;
            ROS_INFO("robot can't reach the goal, try to go unload again");
            ROS_INFO_STREAM("GOAL: x:"<<throwGoodsPoint.target_pose.pose.position.x<<" y:"<<throwGoodsPoint.target_pose.pose.position.y<<" z:"<<throwGoodsPoint.target_pose.pose.orientation.z);
            break;
        }
        case unloadtoload:
        {
            moveBaseActionClientPtr->sendGoal(getGoodsPoint,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            robotState.robotlocation = unloadtoload;
            robotState.goalstate = active;
            ROS_INFO("robot can't reach the goal, try to go load again");
            ROS_INFO_STREAM("GOAL: x:"<<getGoodsPoint.target_pose.pose.position.x<<" y:"<<getGoodsPoint.target_pose.pose.position.y<<" z:"<<getGoodsPoint.target_pose.pose.orientation.z);

            break;
        }
        case loadingtotfl:
        {
            moveBaseActionClientPtr->sendGoal(trafficLightStopLine,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            robotState.robotlocation = loadingtotfl;
            robotState.goalstate = active;
            ROS_INFO("robot can't reach the goal, try to go to tfl again");
            ROS_INFO_STREAM("GOAL: x:"<<trafficLightStopLine.target_pose.pose.position.x<<" y:"<<trafficLightStopLine.target_pose.pose.position.y<<" z:"<<trafficLightStopLine.target_pose.pose.orientation.z);
            break;
        }
        case unloadtoroadline:{
            moveBaseActionClientPtr->sendGoal(lineStart,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            robotState.robotlocation = unloadtoroadline;
            robotState.goalstate = active;
            ROS_INFO("robot can't reach the goal, try to go to roadline again");
            ROS_INFO_STREAM("GOAL: x:"<<lineStart.target_pose.pose.position.x<<" y:"<<lineStart.target_pose.pose.position.y<<" z:"<<lineStart.target_pose.pose.orientation.z);
            break;
        }
        case unloadtostart:{
            moveBaseActionClientPtr->sendGoal(startPoint,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            robotState.robotlocation = unloadtostart;
            robotState.goalstate = active;
            ROS_INFO("robot can't reach the goal, try to go to start area again");
            ROS_INFO_STREAM("GOAL: x:"<<startPoint.target_pose.pose.position.x<<" y:"<<startPoint.target_pose.pose.position.y<<" z:"<<startPoint.target_pose.pose.orientation.z);
            break;
        }
        default:
        {
            ROS_ERROR_NAMED("TCP_Sender", "something worong happened, robot have reach an unknown area!");
            break;
        }
        }
    }
    else if(robotState.goalstate == active)
    {
        //正在执行goal
        //检测道路线

        //这边是变前进变检测车道线，不太稳定，已经改成了前往车道线出发点的方法
        // ROS_INFO_STREAM("active:");
        if(false)//if havedetectedRL
        {
            moveBaseActionClientPtr->sendGoal(lineStart,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2));
            robotState.robotlocation = unloadtoroadline;
            robotState.goalstate = active;
            ROS_INFO("detect road line firstly and chanage goal to go to roadline");
            ROS_INFO_STREAM("GOAL: x:"<<lineStart.target_pose.pose.position.x<<" y:"<<lineStart.target_pose.pose.position.y<<" z:"<<lineStart.target_pose.pose.orientation.z);
        }
    }
    else if(robotState.goalstate == lost)
    {
        ROS_INFO("this is the first goal, robot will go to load area");
        moveBaseActionClientPtr->sendGoal(getGoodsPoint,boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2),boost::bind(&TCP_Sender::GoalActiveCB,this));
        robotState.goalstate = active;
        robotState.robotlocation = starttoload;
        ROS_INFO_STREAM("GOAL: x:" << getGoodsPoint.target_pose.pose.position.x << " y:" << getGoodsPoint.target_pose.pose.position.y << " z:" << getGoodsPoint.target_pose.pose.orientation.z);
    }
    else{
        ROS_ERROR_NAMED("TCP_Sender", "something worong happened, robot have been an unknown goal state!");

    }
}


void TCP_Sender::CloseSocket(){
    close(shCli);
    close(shSrv);

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

    //reuse addr 防止服务端终止后的 timewait状态
    int optval = 1;
    setsockopt(shSrv, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    
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

void TCP_Sender::roadLineControl()
{
    float k = 0.80837;//系数
    float b = 3.48970;
    float angle = robotState.roadLinePianyi*k + b;
    geometry_msgs::Twist cmdData;
    cmdData.linear.x = 0.3;
    cmdData.angular.z = angle/180*3.1415926;
    cmdvelPuber.publish(cmdData);     
}

void TCP_Sender::StopVisonControl()
{
    ROS_INFO("*********************out road line*****************8");
    // while(1);
    qingzhou_bringup::app req;
    req.request.statue = 2;
    if (visioncontrolclient.call(req))
    {
        ROS_INFO("call vision stop success");
        robotState.robotlocation = roadlineout;
        robotState.inRoadLine = false;
        robotStatusMsg.roadlineStatus = 3;

        //返回导航控制状态
        qingzhou_bringup::app req;
        req.request.statue = 0;
        if (visioncontrolclient.call(req))
        {
            //say something
        }
    }
    else
    {
        ROS_INFO("call vision stop control service failed，you may stop it by hand");
        //如果请求服务失败，robotState.inRoadLine 依然被设置为true，依然会进行退出车道线的判断
    }
}

void TCP_Sender::ClearCostmapFirstly() {
    std_srvs::Empty req;
    clearCostmapFirstlyClient.waitForExistence();
    if (clearCostmapFirstlyClient.call(req))
    {
        ROS_INFO_NAMED("TCP_Sender", "Clear Costmap success!");
    }
    else
    {
        ROS_INFO_NAMED("TCP_Sender", "Clear Costmap failed!");
    }

}

void TCP_Sender::_PrintCurruentLocation() {
    switch (robotState.robotlocation)
    {
    case  load: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is 装货区");
    break;
    }
    case loadingtotfl: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is 装货区到红绿灯停止线的途中");break;}
    case tfltounload: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is 红绿灯停止线到卸货区的途中");break;}
    case unload: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is 卸货区");break;}
    case tfl: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is 红绿灯停止线");break;}
    case start: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is 起始区");break;}
    case starttoload:{ROS_INFO_NAMED("TCP_Sender","TCP_Sender:Curruent location is 起始区到装货区的途中");break;}
    case unknow: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is 未知");break;}
    case unloadtoroadline: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is 卸货区到车道线开始点的途中");break;}
    case roadline: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is 车道线");break;}
    case unloadtostart: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is 卸货区到起始区的途中");break;}
    case roadlineout: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is 车道线外面");break;}
    default:
        break;
    }
}    

std::string TCP_Sender::_EmumTranslator(ROBOTLOCATION value){
    if (typeid(value) == typeid(ROBOTLOCATION)) {
        switch (robotState.robotlocation)
        {
        case  load: 
        {
            return "装货区";
            break;
        }                  
        case loadingtotfl: {return "装货区到红绿灯停止线的途中";break;}
        case tfltounload: {return "红绿灯停止线到卸货区的途中" ;break;}
        case unload: {return "卸货区";break;}
        case tfl: {return "红绿灯停止线";break;}
        case start: {return "起始区";break;}
        case starttoload: {return "起始区到装货区的途中";break;}
        case unknow:{return "未知";break;}
        case unloadtoroadline: {return "卸货区到车道线开始点的途中";break;}
        case roadline: {return "车道线";break;}
        case unloadtostart: {return "卸货区到起始区的途中";break;}
        case roadlineout: {return "车道线外面";break;}
        default:
            break;
        }
    }
    else if(typeid(value) == typeid(int))
    {
        // switch (expression)
        // {
        // case /* constant-expression */:
        //     /* code */
        //     break;
        
        // default:
        //     break;
        // }
    }
}

bool TCP_Sender::SwitchAutoGoalControlFlag()
{
    if (this->robotState.autoGoalControl)
    {
        this->robotState.autoGoalControl = false;
        robotStatusMsg.robotstateMsg = this->robotState;//更新robotstate到上位机

    }
    else{
        this->robotState.autoGoalControl = true;
        robotStatusMsg.robotstateMsg = this->robotState;//更新robotstate到上位机
    }
    return this->robotState.autoGoalControl;
}
bool TCP_Sender::SwitchVisionControl(){
    //todo 添加上位机log消息
    // robotState.robotlocation = roadline;
    if (robotState.inRoadLine)//注意：如果让robot处于roadline的location的时候关闭视觉控制，它自动会再开起来，直到退出车道线，在其他地方开启视觉控制可能会引发不可预料的后果
    {
        qingzhou_bringup::app req;
        req.request.statue = 2;
        if (visioncontrolclient.call(req))
        {
            robotState.inRoadLine=false;//神经网络接手控制
            robotStatusMsg.roadlineStatus = 3;
            ROS_INFO_NAMED("TCP_Sender", "stop road line vision control success");
            robotStatusMsg.robotstateMsg = this->robotState;//更新robotstate到上位机
            return true;
        }
        else
        {
            robotStatusMsg.robotstateMsg = this->robotState;//更新robotstate到上位机
            ROS_INFO_NAMED("TCP_Sender", "stop road line vision control failed");
            //todo
            return false;
        }
    }
    else{
        qingzhou_bringup::app req;
        req.request.statue = 1;
        if (visioncontrolclient.call(req))
        {
            robotState.inRoadLine=true;//神经网络接手控制
            robotStatusMsg.roadlineStatus = 2;
            ROS_INFO_NAMED("TCP_Sender", "open road line vision control success");
            robotStatusMsg.robotstateMsg = this->robotState;//更新robotstate到上位机
            return true;
        }
        else
        {
            ROS_INFO_NAMED("TCP_Sender", "open road line vision control failed");
            robotStatusMsg.robotstateMsg = this->robotState;//更新robotstate到上位机
            return false;
            //todo
        }
    }


}

bool TCP_Sender::SwitchTflControl()
{
    if(robotState.openTflDet)
    {
        ROS_INFO_NAMED("robot will close tfl detector, if robot in the tfl location, it may open again by itself");
        qingzhou_bringup::app req;
        req.request.statue = 4;
        if (visioncontrolclient.call(req))
        {
            robotState.openTflDet = false;
            ROS_INFO_NAMED("RCP_Sender", "close color detector success");
            robotStatusMsg.robotstateMsg = this->robotState;//更新robotstate到上位机
            return true;
        }
        else
        {
            //todo
            ROS_INFO_NAMED("RCP_Sender", "close color detector failed");
            robotStatusMsg.robotstateMsg = this->robotState;//更新robotstate到上位机
            return false;
        }
    }
    else{
        ROS_INFO_NAMED("robot will open tfl detector, 如果机器人检测到红色/绿色他可能会自己前往下一个目标点，请注意不要在不正确的地方开启这个东西");
        qingzhou_bringup::app req;
        req.request.statue = 3;
        if (visioncontrolclient.call(req))
        {
            robotState.openTflDet = true;
            ROS_INFO_NAMED("RCP_Sender", "open color detector success");
            robotStatusMsg.robotstateMsg = this->robotState;//更新robotstate到上位机
            return true;
        }
        else
        {
            //todo
            ROS_INFO_NAMED("RCP_Sender", "open color detector failed");
            robotStatusMsg.robotstateMsg = this->robotState;//更新robotstate到上位机
            return false;
        }
    }
}