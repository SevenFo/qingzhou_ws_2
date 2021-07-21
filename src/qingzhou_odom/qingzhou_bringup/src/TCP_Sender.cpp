#include "TCP_Sender.h"
#include "math.h"

TCP_Sender::TCP_Sender(const ros::NodeHandle &nodeHandler)
{
    nh = nodeHandler;//获取节点句柄

    nh.param("time1",_ppstime1,4.0f);
    nh.param("time2",_ppstime2,4.0f);
    nh.param("speed1_x",_ppsspeed1x,0.5f);
    nh.param("speed1_z",_ppsspeed1z,-0.8f);
    nh.param("speed2_x",_ppsspeed2x,0.5f);
    nh.param("speed2_z",_ppsspeed2z,0.8f);

    _redStartTime = ros::Time(0);
    //订阅
    bettarySuber = nh.subscribe<std_msgs::Float32>("/battery",5,&TCP_Sender::SubBettaryInfoCB,this);//电池电量
    speedSuber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",50,&TCP_Sender::SubSpeedCB,this);//速度
    trafficLightSuber = nh.subscribe<geometry_msgs::Vector3>("/pianyi",1,&TCP_Sender::SubTrafficLightCB,this);//红绿灯
    pianyiSuber = nh.subscribe<geometry_msgs::Vector3>("/pianyi",1,&TCP_Sender::SubLineCB,this);
    //service client
    visioncontrolclient = nh.serviceClient<qingzhou_bringup::app>("/vision_control");
    dynamicparamsclient = nh.serviceClient<qingzhou_bringup::app>("/DynamicParamsClient");
    clearCostmapFirstlyClient = nh.serviceClient<std_srvs::Empty>("/clear_cost_map");
    clearCostmapClient = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    //service server
    appServiceServer = nh.advertiseService("/TCP_Sender/app",&TCP_Sender::AppServiceCB,this);
    //Publisher
    cmdvelPuber = nh.advertise<geometry_msgs::Twist>("/cmd_vel",50);
    _goalStatusPuber = nh.advertise<actionlib_msgs::GoalStatus>("/TCP_Sender/GoalStatus", 1);
    _locationInMapPuber = nh.advertise<geometry_msgs::Pose>("/TCP_Sender/PoseInMap", 2);
    //Action Client
    moveBaseActionClientPtr = new MoveBaseActionClient("move_base");
    //Timre
    updateStateTimer = nh.createTimer(ros::Duration(0.065),boost::bind(&TCP_Sender::UpdateStateTimerCB,this),false,false);

    ROS_INFO_NAMED("TCP_Sender", "Waiting services");
    clearCostmapClient.waitForExistence();

    //Thread
    this->_tfListenThread = new boost::thread(boost::bind(&TCP_Sender::ListenRobotPose, this,boost::ref(this->robotPose))); //开启监听pose的线程
    // this->_watchRLStart = new boost::thread(boost::bind(&TCP_Sender::WatchRLStartAndCancleGoal, this, this->moveBaseActionClientPtr,&this->cmdvelPuber));

    robot_local_state = rls();

    this->haveDetectedRedTfl = false;
    haveDetectedGreenTfl = false;
    this->_circleTimeList.clear();

    //*********init point position************
    this->_roadlinestartPose.position.x = 0.2725;
    this->_roadlinestartPose.position.y = -4.338;


    sleepDur = ros::Duration(1);
    this->_open_debug = true;
}

TCP_Sender::~TCP_Sender()
{
    delete moveBaseActionClientPtr;
    delete _tfListenThread;
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
    this->updateStateTimer.start();
    if (ConvertToUnlocked()) //转化成非阻塞
        return true;
    else
        return false;
}

void TCP_Sender::CloseSocket(){
    close(shCli);
    close(shSrv);

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


bool TCP_Sender::AppServiceCB(qingzhou_bringup::app::Request &req,qingzhou_bringup::app::Response &res)
{
    if(req.statue == 1)
    {
        ROS_INFO_STREAM_COND(_open_debug, "weak _watchRLCond");
        this->_watchRLCond.notify_one();
        return true;
    }
    return false;
}

//一直唤醒
void TCP_Sender::SubBettaryInfoCB(const std_msgs::Float32::ConstPtr &msg)
{
    robotStatusMsg.bettary = msg.get()->data;
   // std::cout<<"log :set bettary value"<<std::endl;

}
//一直唤醒
void TCP_Sender::SubSpeedCB(const geometry_msgs::Twist::ConstPtr &msg)
{
    robotStatusMsg.linearSpeed =msg.get()->linear.x;
    robotStatusMsg.angularSpeed=msg.get()->angular.z;
}

//v2 只有开启的时候才唤醒
void TCP_Sender::SubTrafficLightCB(const geometry_msgs::Vector3::ConstPtr &msg)
{
    // robotStatusMsg.trafficLight = (int)(msg.get()->x);
    
    this->robot_local_state.trafficLightColor = (TRAFFICLIGHT)((int)(msg.get()->x));
    // ROS_INFO_STREAM_NAMED("SubTrafficLightCB", "traffic light:" << this->robot_local_state.trafficLightColor);
    if ((this->robot_local_state.trafficLightColor == red || this->robot_local_state.trafficLightColor == yellow) && (!this->haveDetectedRedTfl))
    {
        ROS_INFO_STREAM("det red");
        _redStartTime = ros::Time::now();//计时6秒
        this->haveDetectedRedTfl = true;//不要频繁发布目标点
        haveDetectedGreenTfl = false;
        this->UpdateRobotLocation(loadingtotfl);
        this->UpdateRobotCurruentGoal(goal_tfl);
        this->robot_local_state.goalState = active;
        this->ExecGoal();
    }
    //检测到绿灯 或者 停车时间超过6s
    else if(((this->robot_local_state.trafficLightColor == green) ||(_redStartTime.toSec()-ros::Time::now().toSec()) > 6)&& (!this->haveDetectedGreenTfl) ) //如果是绿灯，不管车在哪里都直接前往下一个目标点
    {
        std::cout <<"now time:"<<ros::Time::now()<< "start time:"<<_redStartTime<<"  " << (ros::Time::now().toSec() - _redStartTime.toSec()) << std::endl;
        _redStartTime = ros::Time(0);
        //频繁发布绿灯的目标点会怎么样？ 會開不起來
        ROS_INFO_STREAM("det green");
        this->haveDetectedGreenTfl = true;
        haveDetectedRedTfl = false;
        this->UpdateRobotLocation(loadtounload);
        this->UpdateRobotCurruentGoal(goal_unload);
        this->robot_local_state.goalState = active;
        this->ExecGoal();
        //等到了unload在关闭红绿灯探测吧

    }
}

//v2 v2 只有开启的时候才唤醒
void TCP_Sender::SubLineCB(const geometry_msgs::Vector3::ConstPtr &msg)
{

    //注意，只有发送请求之后，这个话题在有值，这个回调函数才会被调用，并且这个函数是为了检测有没有退出赛道

    int pianyi = (msg.get()->y);
    // std::cout << "painyi: " << pianyi << std::endl;
    if (int(pianyi) && this->robot_local_state.openRoadLineDet) //pianyi >0 才进入 因为和红绿灯共同使用一个话题，默认情况下painyi=0
    {

        if(pianyi >998 ){//偏移跳变大于30判断退出赛道 不太好用 废弃了
            this->UpdateRobotLocation(roadlineout);
            if (this->StopRLDet())
            {
                std::cout << "*********sleep*************" << std::endl;
                ros::Duration(1.0).sleep();
                std::cout << "*********set reach*************" << std::endl;
                this->robot_local_state.goalState = reach;
            }
            else{
                ROS_INFO_NAMED("TCP_Sender", "Oh no! robot cant stop!!");
            }
            //
            
        }
    }
    else{
    }
}
//目标点完成的回调函数用于更改 goalstate v2
void TCP_Sender::GoalDoneCB(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
    ROS_INFO("goan done result: [%s]", state.toString().c_str());
    // first clear costmap
    
    //ROS_INFO("Answer: %i", result->sequence.back());
    //出发点-转载点 -> 装载点； 装载点 - 交通灯->交通灯； 交通灯 - 卸货点->卸货点； 卸货点 - 车道线出发点->车道线出发点 ->
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if(this->robot_local_state.location != unloadtostart)
            this->ClearCostmapAndWait();
        // haveDetectedTfl = false;
        this->robot_local_state.goalState = reach;
        switch (this->robot_local_state.location)
        {
        case starttoload: {
            this->UpdateRobotLocation(load);
            break;
            }
        case tfltounload:{this->UpdateRobotLocation(unload);break;}
        case loadtounload:{this->UpdateRobotLocation(unload);break;}
        case unloadtostart:{
            this->UpdateRobotLocation(start);
            this->_circleTimeList.push_back((ros::Time::now()-this->_startTime).toSec());
            std::cout<<"|||||||||CIRCUL TIME:"<<this->_circleTimeList.back()<<"|||||||||"<<std::endl;
            std::cout << "-----------------TIME COUNTER----------------"<<std::endl;
            this->_startTime = ros::Time::now();
            for (int i = 0; i < this->_circleTimeList.size();i++)
                std::cout << " | " << i << " circle:" << this->_circleTimeList.at(i) << std::endl;
            std::cout << "-----------------============----------------"<<std::endl;
            break;
            }
        case loadingtotfl:{this->UpdateRobotLocation(tfl);break;}
        case unloadtoroadline:
        {
            this->UpdateRobotLocation(roadlinestart);
            break;
        }
        default:{ROS_WARN("cant get positonstate");break;}
        }
    }
    else if(state == actionlib::SimpleClientGoalState::ABORTED || state == actionlib::SimpleClientGoalState::PREEMPTED)
    {
        this->ClearCostmapAndWait();
        this->robot_local_state.goalState = aborted;
    }
    else
    {
        
    }
    // else if(state == actionlib::SimpleClientGoalState::PREEMPTED)
    // {
    //     this->robot_local_state.goalState = active;//可能需要修改
    // }
}
void TCP_Sender::GoalActiveCB()
{    
    std::cout<<"goal active"<<std::endl;
}



bool TCP_Sender::SendRobotStatusInfo()//发送机器人当前的状态
{
    return SendMsg(&robotStatusMsg,sizeof(robotStatusMsg));
}



void TCP_Sender::roadLineControl()
{
    float k = 0.80837;//系数
    float b = 3.48970;
    // float angle = robotState.roadLinePianyi*k + b;
    geometry_msgs::Twist cmdData;
    cmdData.linear.x = 0.3;
    // cmdData.angular.z = angle/180*3.1415926;
    cmdvelPuber.publish(cmdData);     
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
    switch (this->robot_local_state.location)
    {
    case  load: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is load");
    break;
    }
    case loadingtotfl: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [load to tfl]");break;}
    case tfltounload: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [tfl to unload]");break;}
    case unload: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [unload]");break;}
    case tfl: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [tfl]");break;}
    case start: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [start]");break;}
    case starttoload:{ROS_INFO_NAMED("TCP_Sender","TCP_Sender:Curruent location is [start to load]");break;}
    case unknow: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [unknow]");break;}
    case unloadtoroadline: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [unload to rl start]");break;}
    case roadline: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [roadline]");break;}
    case unloadtostart: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [unload to start]");break;}
    case roadlineout: {ROS_INFO_NAMED("TCP_Sender", "TCP_Sender:Curruent location is [road line out]");break;}
    default:
    {
        ROS_INFO_NAMED("TCP_Sender", "robot go to somewhere unkown");
        break;
    }
    }
}    

std::string TCP_Sender::_EmumTranslator(ROBOTLOCATION value){
    // std::cout << "location value" << std::endl;
    if (true)
    {
        switch (value)
        {
        case  load: 
        {
            return "[load]";
            break;
        }                  
        case loadingtotfl: {return "[load to tfl]";break;}
        case tfltounload: {return "[tlf to unload]" ;break;}
        case unload: {return "[unload]";break;}
        case tfl: {return "[tfl]";break;}
        case start: {return "[start]";break;}
        case starttoload: {return "[start to load]";break;}
        case unknow:{return "[unknow]";break;}
        case unloadtoroadline: {return "[unload to rl start]";break;}
        case roadline: {return "[road line]";break;}
        case unloadtostart: {return "[unload to start]";break;}
        case roadlineout: {return "[road line out]";break;}
        case loadtounload: {return "[road to unload]";break;}
        case roadlinestart:{return "[roadlinestart]";break;}
        default:
            {return "[???]";break;}
            break;
        }
    }
}
std::string TCP_Sender::_EmumTranslator(ROBOTGOALPOINT goal)
{
    switch (goal)
    {
    case  goal_load:
    {
        return "[load]";
        break;
    }
    case goal_unload: {return "[unload]";break;}
    case goal_tfl: {return "[tfl]";break;}
    case goal_start: {return "[start]";break;}
    case goal_roadlinestart: {return "[road line start]";break;}
    default:
    {return "[???]";break;}
        break;
    }
}

//v2
bool TCP_Sender::SwitchAutoGoalControlFlag()
{
    if (this->robot_local_state.autoGoalControl)
    {
        ROS_INFO_NAMED("TCP_Sender", "auto goal control shutdown!");
        this->robot_local_state.autoGoalControl = false;
    }
    else{
        ROS_INFO_NAMED("TCP_Sender", "auto goal control active");
        this->robot_local_state.autoGoalControl = true;
    }
    return this->robot_local_state.autoGoalControl;
}

//v2
bool TCP_Sender::SwitchVisionControl(){
    //todo 添加上位机log消息

    if(this->robot_local_state.openRoadLineDet)
    {
        // ROS_INFO_NAMED("TCP_Sender", "att: RL control will be shutdown");
        this->StopRLDet();
        this->_PrintCurruentLocation();
        return true;
    }
    else{
        // ROS_INFO_NAMED("TCP_Sender", "att: RL control will be active");
        this->OpenRLDet();
        this->_PrintCurruentLocation();
        return true;
    }
}
//v2
bool TCP_Sender::SwitchTflControl()
{

    if(this->robot_local_state.openTflDet)
    {
        // ROS_INFO_NAMED("TCP_Sender", "att: TFL control will be shutdown");
        this->StopTflDet();
        return true;
    }
    else
    {
        // ROS_INFO_NAMED("TCP_Sender", "att: TFL control will be active");
        this->OpenTflDet();
        return true;
    }
}

bool TCP_Sender::OpenTflDet()
{
    ROS_INFO_NAMED("TCP_Sender","OPENING TFL detector...");
    qingzhou_bringup::app req;
    req.request.statue = 3;
    if (visioncontrolclient.call(req))
    {
        ROS_INFO_NAMED("RCP_Sender", "open success");
        this->robot_local_state.openTflDet = true;
        return true;
    }
    else
    {
        ROS_INFO_NAMED("RCP_Sender", "open failed");
        this->robot_local_state.openTflDet = false;
        return false;
    }
}

bool TCP_Sender::StopTflDet()
{
    ROS_INFO_NAMED("TCP_Sender","STOPING TFL detector...");
    qingzhou_bringup::app req;
    req.request.statue = 4;
    if (visioncontrolclient.call(req))
    {
        ROS_INFO_NAMED("RCP_Sender", "stop success");
        this->robot_local_state.openTflDet = false;
        return true;
    }
    else
    {
        ROS_INFO_NAMED("RCP_Sender", "stop failed");
        this->robot_local_state.openTflDet = true;
        return false;
    }
}

bool TCP_Sender::OpenRLDet()
{
    qingzhou_bringup::app req;
    req.request.statue = 1;
    if(visioncontrolclient.call(req))
    {
        this->robot_local_state.openRoadLineDet = true;
        this->UpdateRobotLocation(roadline);
        ROS_INFO("open RL control success");
        return true;
    }
    else
    {
        this->robot_local_state.openRoadLineDet = false;
        ROS_INFO("open RL control failed");
        return false;
    }
}

bool TCP_Sender::StopRLDet()
{
    qingzhou_bringup::app req;
    req.request.statue = 2;
    if (visioncontrolclient.call(req))
    {
        ROS_INFO("stop RL control success");
        this->robot_local_state.openRoadLineDet = false;
        //等待一段时间后 返回导航控制状态
        ROS_INFO_NAMED("TCP_Sender_stopRLDet","SLEEP 1s (make robot turn right)");
        ros::Duration(1).sleep();
        qingzhou_bringup::app req;
        req.request.statue = 0;
        if (visioncontrolclient.call(req))
        {
            //say something
        }
        return true;
    }
    else
    {
        ROS_INFO("stop RL control failed");
        this->robot_local_state.openRoadLineDet = true;
        //如果请求服务失败，robotState.inRoadLine 依然被设置为true，依然会进行退出车道线的判断
        return false;
    }
}

void TCP_Sender::ExecGoal()
{

    //**********
    // //计时
    // std::cout<<"|||||||||PART TIME:"<<ros::Time::now()-this->_tmpStartTime<<"|||||||||"<<std::endl;
    // this->_tmpStartTime = ros::Time::now();
    //************

    move_base_msgs::MoveBaseGoal tmp;
    // move_base_msgs::MoveBaseActionGoal tmpwgoalid;
    // tmpwgoalid.header.frame_id = "map";
    // tmpwgoalid.header.stamp = ros::Time::now();
    // tmpwgoalid.goal_id.id = this->_EmumTranslator(robot_local_state.curruentGoal);
    // tmpwgoalid.goal_id.stamp = ros::Time::now();
    tmp.target_pose.header.frame_id = "map";
    tmp.target_pose.pose.position.x = robot_local_state.goalList[robot_local_state.curruentGoal].x;
    tmp.target_pose.pose.position.y = robot_local_state.goalList[robot_local_state.curruentGoal].y;
    tmp.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_local_state.goalList[robot_local_state.curruentGoal].z);
    // tmpwgoalid.goal = tmp;
    ROS_INFO_STREAM_NAMED("TCP_Sender", "goal setting: "<<this->_EmumTranslator(robot_local_state.curruentGoal)<<" and detail:"<<"x:"<<tmp.target_pose.pose.position.x<<" y:"<<tmp.target_pose.pose.position.y<<" z:"<<robot_local_state.goalList[robot_local_state.curruentGoal].z<<"qx:"<<tmp.target_pose.pose.orientation.x<<" qy:"<<tmp.target_pose.pose.orientation.y<<" qz"<<tmp.target_pose.pose.orientation.z<<" qw"<<tmp.target_pose.pose.orientation.w);

    tmp.target_pose.header.stamp = ros::Time::now();
    this->robot_local_state.goalState = active;
    // this->moveBaseActionClientPtr->sendGoal(tmp, boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2), boost::bind(&TCP_Sender::GoalActiveCB, this));
    this->moveBaseActionClientPtr->sendGoal(tmp, boost::bind(&TCP_Sender::GoalDoneCB, this, _1, _2), boost::bind(&TCP_Sender::GoalActiveCB, this));
    // this->moveBaseActionClientPtr->sendGoal()
    ROS_INFO_NAMED("TCP_Sender", "send goal point success");


    // this->moveBaseActionClientPtr->cancelAllGoals();
}

void TCP_Sender::UpdateRobotLocation(ROBOTLOCATION value)
{
    this->robot_local_state.location = value;
    ROS_INFO_STREAM_NAMED("TCP_Sender", "LOCATION update: "<<this->_EmumTranslator(this->robot_local_state.location)<<" success");
}

void TCP_Sender::UpdateRobotCurruentGoal(ROBOTGOALPOINT goal)
{
    this->robot_local_state.curruentGoal = goal;
    ROS_INFO_STREAM_NAMED("TCP_Sender", "GOAL update: "<<this->_EmumTranslator(this->robot_local_state.curruentGoal)<<"success");
}

void TCP_Sender::RunGoal_v2()
{
    //如果成功到达某个目标点那就前往下一个目标点
    //出发点 -> 装载点 -> 交通灯停止线 -> （由话题回调函数控制，在交通灯停止线等待绿灯） -> 卸载区 ->
    //(车道线 -> 发送请求视觉控制 ->（若请求成功则开始检测是否退出车道线） -> 车道线外面: 这个过程中状态都是reach) -> 出发点
    // ROS_INFO_STREAM("run goal state:"<<robotState.goalstate);
    if(this->robot_local_state.goalState == reach)//成功到达某个目标点
    {
        // ROS_INFO_STREAM("********robot reach goal point success!*********");

        this->robot_local_state.goalState = active;//reach状态只能进入一次（每次goalreach）

        switch (this->robot_local_state.location)
        {
        case start:
        {
            this->_countTimeList.push_back((ros::Time::now() - this->_tmpStartTime).toSec());
            std::cout<<"|||||||||PART TIME([unload to start]):"<<this->_countTimeList.back()<<"|||||||||"<<std::endl;
            this->_tmpStartTime = ros::Time::now();
            for (int i = 0; i < this->_countTimeList.size();i++)
                std::cout << " | " << i << " part:" << this->_countTimeList.at(i) << std::endl;
            this->_countTimeList.clear();
            //前往getGood point
            this->UpdateRobotLocation(starttoload);
            this->UpdateRobotCurruentGoal(goal_load);
            this->ExecGoal();
            break;
        }
        case load:
        {
            this->_countTimeList.push_back((ros::Time::now() - this->_tmpStartTime).toSec());
            std::cout<<"|||||||||PART TIME([start to load]):"<<this->_countTimeList.back()<<"|||||||||"<<std::endl;
            this->_tmpStartTime = ros::Time::now();


            if (this->robot_local_state.autoGoalControl == true) //只有开启autogoalcontrol之后才可以一到达装货区就发布去停止线的goal，否则要我们手动发布
            {
                this->UpdateRobotLocation(loadtounload);
                this->UpdateRobotCurruentGoal(goal_unload);
                this->ExecGoal();
            }
            else{
                ROS_INFO_NAMED("TCP_Sender", "robot have been to load, pls pub next goal");
            }
            //开启红绿灯探测
            this->OpenTflDet();
            break;
        }
        case tfl:
        {
            //todo
            //如果到达停止线说明现在一定是红灯，没有别的情况会这样了
            ROS_INFO_NAMED("TCP_Sender", "robot have reached tfl，waiting green...");
            break;
        }
        case unload:
        {
            this->_countTimeList.push_back((ros::Time::now() - this->_tmpStartTime).toSec());
            std::cout<<"|||||||||PART TIME([load to unload]):"<<this->_countTimeList.back()<<"|||||||||"<<std::endl;
            this->_tmpStartTime = ros::Time::now();
            if(this->robot_local_state.autoGoalControl)//只有开启autogoalcontrol之后才可以一到卸货区就发布去roalline的goal，否则要我们手动发布
            {
                this->UpdateRobotLocation(unloadtoroadline);
                this->UpdateRobotCurruentGoal(goal_roadlinestart);
                this->ExecGoal();
            }
            else{
                ROS_INFO_NAMED("TCP_Sender", "robot have reached unload, pls pub next goal");
            }
            qingzhou_bringup::app req;
            req.request.statue = 1;
            this->dynamicparamsclient.call(req);
            ROS_INFO_NAMED("TCP_Sender", "DYNAMIC PARAPMS OPEN");
            //关闭红绿灯探测
            this->StopTflDet();
            // this->OpenRLDet();
            break;
        }
        case roadlinestart:
        {
            //开启车道线探测
            this->OpenRLDet();//顺便改状态
            break;
        }
        //todo...............
        case roadline:
        {
            ROS_INFO_NAMED("TCP_Sender", "robot is controlled by vision...");
            //视觉接管控制
            break;
        }
        case roadlineout:
        {
            this->UpdateRobotLocation(unloadtostart);
            this->UpdateRobotCurruentGoal(goal_start);
            this->ExecGoal();
            // this->ExecGoal();
            break;
        }

        default:
        {
            //print curruent location todo
            ROS_ERROR_NAMED("TCP_Sender", "something worong happened, robot have reach an unknown area!");
            break;
        }
        }   
        
    }
    else if(this->robot_local_state.goalState == aborted) //如果目标在中途异常停止
    {
        
        // ROS_INFO_STREAM("abort:");
        this->robot_local_state.goalState = active;//reach状态只能进入一次（每次goalreach）
        ROS_INFO_STREAM_NAMED("TCP_Sender", "can' reach goal point, will send goal again");
        this->_PrintCurruentLocation();//打印当前的坐标

        //正常情况下这个情况下机器人的location和goal都不会发生变化，应该不需要更新，直接再发布一次目标点就行

        this->ExecGoal();
    }
    else if(this->robot_local_state.goalState == active)
    {
        //正在执行goal

        //当机器人开出去车道线但一直无法停下来，可能会一直处于active的状态，需要进行处理
    }
    else if(this->robot_local_state.goalState == lost)
    {
        this->robot_local_state.goalState = active;
        ROS_INFO("this is the first goal point, robot will go to the load area");
        this->_PrintCurruentLocation();//打印当前的区域
        this->UpdateRobotLocation(starttoload);
        this->UpdateRobotCurruentGoal(goal_load);
        this->_PrintCurruentLocation();//打印当前的区域
        this->_startTime = ros::Time::now();
        this->_tmpStartTime = this->_startTime;
        this->ExecGoal();
        }
    else{
        ROS_ERROR_NAMED("TCP_Sender", "robot curruent goal point known, this should not happen!"); 
    }
}

bool TCP_Sender::ClearCostmapAndWait()
{
    std_srvs::Empty req;
    if (clearCostmapClient.call(req)) {
        ROS_INFO_NAMED("TCP_Sender", "clear costmap success!");
        //在转角的时候路径规划没有考虑障碍物？
        ROS_INFO_NAMED("TCP_Sender", "Sleep 1.0s wating costmap)");
        sleepDur = ros::Duration(1.0);
        sleepDur.sleep();
        ROS_INFO_NAMED("TCP_Sender", "weak up");
        return true;
    }
    else
    {
        ROS_INFO_NAMED("TCP_Sender", "clear costmap failed");
        return false;
    }
}


void TCP_Sender::UpdateRobotGoalList(point3d *goalList)
{

    //todo 添加判断是否更新了列表


    ROS_INFO_NAMED("TCP_Sender", "robot are trying to update goal List...");
    for (int i = 0; i < 5;i++)
    {
        this->robot_local_state.goalList[i] = goalList[i];

        // ROS_INFO_STREAM("goal list information:" << goalList[i].x << " " << goalList[i].y << " " << goalList[i].z);
        ROS_INFO_STREAM("goal list information:" << this->robot_local_state.goalList[i].x << " " << this->robot_local_state.goalList[i].y << " " << this->robot_local_state.goalList[i].z);
    }
    ROS_INFO_NAMED("TCP_Sender", "update success!");
}

void TCP_Sender::UpdateStateTimerCB()
{
    robotStatusMsg.goalstate = robot_local_state.goalState;
    robotStatusMsg.robotlocation = robot_local_state.location;
    robotStatusMsg.autoGoalControl = robot_local_state.autoGoalControl;
    robotStatusMsg.openTflDet = robot_local_state.openTflDet;
    robotStatusMsg.openRoadLineDet = robot_local_state.openRoadLineDet;
    robotStatusMsg.trafficLight = robot_local_state.trafficLightColor;
    robotStatusMsg.locationInMapY = this->robotPose.position.y;
    robotStatusMsg.locationInMapX = this->robotPose.position.x;

    actionlib_msgs::GoalStatus gs;
    gs.goal_id.stamp = ros::Time::now();
    gs.goal_id.id = this->_EmumTranslator(robot_local_state.curruentGoal);
    gs.status = robot_local_state.goalState;

    this->_goalStatusPuber.publish(gs);
    this->_locationInMapPuber.publish(this->robotPose);

    // ros::Duration
    // ROS_INFO_STREAM_NAMED("tcp_sender:", "robotpose: x:" << this->robotPose.position.x << " y:" << this->robotPose.position.y);
}

void TCP_Sender::_RunSpeedPlan()
{
    std::vector<SpeedPlanPoint> _sp;
    _sp.push_back(SpeedPlanPoint(_ppstime1,0,0,_ppsspeed1x,_ppsspeed1z));
    _sp.push_back(SpeedPlanPoint(_ppstime2,0,0,_ppsspeed2x,_ppsspeed2z));
    _sp.push_back(SpeedPlanPoint(0.1,0,0,0,0));

    for(auto spp: _sp)
    {
        cmdvelPuber.publish(spp._speed);
        spp._dua.sleep();
    }
}

void TCP_Sender::ListenRobotPose(geometry_msgs::Pose &robotPose)
{

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listen(tfBuffer);
    geometry_msgs::TransformStamped tfs;
    ros::Rate rate(15);
    boost::mutex robotposeMutex;
    // robotposeMutex.unlock();
    std::string err;
    while (ros::ok())
    {

        if(tfBuffer.canTransform("base_link", "map",ros::Time(0),&err))
        {
            tfs = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
            robotposeMutex.lock();
            robotPose.position.x = tfs.transform.translation.x;
            robotPose.position.y = tfs.transform.translation.y;
            robotPose.orientation = tfs.transform.rotation;
            robotposeMutex.unlock();
            // ROS_INFO_STREAM_NAMED("tcp_sender:", "robotpose_thread: x:" << tfs.transform.translation.x << " y:" << tfs.transform.translation.y);

        }
        else
        {
            //cant trasform
            // std::cout << "tferr:" << err<<std::endl;
        }
        rate.sleep();
    }
}

void TCP_Sender::WatchRLStartAndCancleGoal(MoveBaseActionClient* client,ros::Publisher* cmdpuber)
{
    ros::Rate rate(20);
    geometry_msgs::Twist speed;
    speed.linear.x = 0.2;
    speed.angular.z = -0.4;
    boost::unique_lock<boost::recursive_mutex> lock(watchRLMutex);
    while (ros::ok())
    {
        //当机器人的目标点没有位于unloadtoroadline的时候就让该线程休眠，进入unloadtoroadline的时候weak才有用
        while (this->robot_local_state.location != unloadtoroadline)
        {
            _watchRLCond.wait(lock);
        }
        // ROS_INFO_STREAM_COND(this->_open_debug, "TCP_Sender: abs(rx - rlx) = "<<abs(robotPose.position.x - 0.46)<<" ry:"<<robotPose.position.y);
        if(abs(robotPose.position.x - 0.46)<0.05 && robotPose.position.y > -4.73)
        {
            client->cancelAllGoals();
            ROS_INFO_STREAM_COND(this->_open_debug, "TCP_Sender: cancel all goal and open RL det");
            
            this->UpdateRobotLocation(roadlinestart);
            this->robot_local_state.goalState = reach;
            cmdpuber->publish(speed);
        }
    }
    rate.sleep();
}
