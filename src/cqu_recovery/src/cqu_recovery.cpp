#include "cqu_recovery/cqu_recovery.h"

PLUGINLIB_EXPORT_CLASS(cqu_recovery_behavior::cqu_recovery, nav_core::RecoveryBehavior)

namespace cqu_recovery_behavior
{
    /*
    movebase流程图
    初始化->开启代价地图循环、开启action server等->executeCb() goal action的回调函数
    『判断目标点的坐标是否有效->进入一个节点循环->随时可以更改循环间隔参数 如果被抢占则判断新的目标是不是能用的，如果可以 就接受新的目标，否则 aborted。接受新的
    目标之后state将被设置为Planning，runplanner设置为true开始执行路径规划 』
    『executeCycle:  』
    『planThread：这是一个新的线程 』
    */

    /**
     * @brief 
     * 
     * @param name 
     * @param tf 
     * @param global_costmap 
     * @param local_costmap 
     */
    void cqu_recovery::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
    {
        if(!isInitialize)
        {
            _name = name;
            _tfPtr = tf;
            _globalCostmapPtr = global_costmap;
            _localCostmapPtr = local_costmap;

            _n = ros::NodeHandle(name);//命名空间解析为 /move_base/name/...
            ROS_INFO_STREAM_COND_NAMED(_debug, "cqu_recovery_init", "cqu_recovery_initialize: node name:" << name);

            _n.param("foot_print", _footlenth, 10);

            this->robotCurruentGoalSuber = _n.subscribe<geometry_msgs::PoseStamped>("/move_base/current_goal", 1, &cqu_recovery::RobotCurruentGoalCB, this);
            this->globalPlanSuber = _n.subscribe<nav_msgs::Path>("/move_base/GlobalPlanner/plan",2,&cqu_recovery::GlobalPlanCB,this);

            _globalKValue = 0;
            isInitialize = true;
            _debug = true;
        }
        else
        {
            ROS_WARN("you have initialized cqu_recovery yet");
        }
    }
    void cqu_recovery::runBehavior()
    {
        if(!isInitialize)
        {
            ROS_ERROR("you haven't initialized cqu_recovery yet");
            return ;
        }
        geometry_msgs::Twist tmpControl;
        geometry_msgs::PoseStamped robotPose = geometry_msgs::PoseStamped();
        if(_globalCostmapPtr->getRobotPose(robotPose))
        {
            tf::Quaternion quat;
            double robotYaw = Degree360(tf::getYaw(robotPose.pose.orientation));
            double K = Degree360(_planDegree);
            ROS_INFO_STREAM_NAMED("cqu_recovery:", "K value:" << K / 3.14159 * 180 << " robotYaw:" << robotYaw / 3.14159 * 180);
            if (K >  robotYaw)
            {
                if(abs(K-robotYaw)>3.1415926)
                {
                    ROS_INFO_STREAM_NAMED("cqu recovery", "cqu recovery: K_value > robotYaw. Z value = +1.0");
                    tmpControl.angular.z = -0.5;
                }
                else
                {
                    ROS_INFO_STREAM_NAMED("cqu recovery", "cqu recovery: K_value > robotYaw. Z value = -1.0");
                    tmpControl.angular.z = +0.5;//can set to be param //倒车好像是相反的
                }
            }
            else{
                //
                if(abs(K-robotYaw)>3.1415926)
                {
                    ROS_INFO_STREAM_NAMED("cqu recovery", "cqu recovery: K_value > robotYaw. Z value = +1.0");
                    tmpControl.angular.z = +0.5;
                }
                else
                {
                    ROS_INFO_STREAM_NAMED("cqu recovery", "cqu recovery: K_value > robotYaw. Z value = -1.0");
                    tmpControl.angular.z = -0.5;//can set to be param //倒车好像是相反的
                }
            }
        }
        else
        {
            tmpControl.angular.z = 0;
        }
        ros::Duration sleepTime(0.5);
        ROS_INFO("action cqu recovery behavious, robot will run back for 0.5 s");
        ros::Publisher cmdVelPub;
        cmdVelPub = _n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        
        tmpControl.linear.x = -0.5;
        cmdVelPub.publish(tmpControl);
        sleepTime.sleep();
        tmpControl.linear.x = 0;
        tmpControl.angular.z = 0;
        cmdVelPub.publish(tmpControl);
        ROS_INFO("cqu recovery behavious over!");
    }

    double cqu_recovery::CalculateKValue(geometry_msgs::Pose startp,geometry_msgs::Pose endp)
    {
        ROS_INFO_STREAM_COND_NAMED(_debug,"cqu recovery","startpx:"<<startp.position.x<<" startpy:"<<startp.position.y<<" endpx"<<endp.position.x<<" endpy:"<<endp.position.y);
        return (endp.position.y - startp.position.y) / (endp.position.x - startp.position.x);
    }
    double cqu_recovery::CalculateKValue(const geometry_msgs::Point startp,const geometry_msgs::Point endp)
    {
        ROS_INFO_STREAM_COND_NAMED(_debug,"cqu recovery","startpx:"<<startp.x<<" startpy:"<<startp.y<<" endpx"<<endp.x<<" endpy:"<<endp.y);
        return (endp.y - startp.y) / (endp.x - startp.x);
    }

    void cqu_recovery::RobotCurruentGoalCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        ROS_INFO_STREAM_COND_NAMED(_debug,"cqu recovery", "get pose:x:"<<msg.get()->pose.position.x<<" y:"<<msg.get()->pose.position.y);
        _robotCurruentGoal = msg.get()->pose;
    }
    void cqu_recovery::GlobalPlanCB(const nav_msgs::PathConstPtr &msg)
    {
        // //取两个点，计算斜率
        if(msg.get()->poses.size() < _footlenth+1)
        {
            ROS_INFO_STREAM_NAMED("cqu_recvoery", "cqu_recovery: global plan too short");
            return;
        }
        _planPoint1 = msg.get()->poses.at(0).pose;
        _planPoint2 = msg.get()->poses.at(_footlenth).pose;

        // ROS_INFO_STREAM_COND_NAMED(_debug, "cqu_recovery_globalplancb", "cqu_recovery_globalplancb:Global plan p1x:" <<_planPoint1.position.x<<" p1y:"<<_planPoint1.position.y <<" p2x:"<<_planPoint2.position.x << "p2y:"<<_planPoint2.position.y);

        _globalKValue = this->CalculateKValue(_planPoint1,_planPoint2);
        _planDegree = atan2(_planPoint2.position.y - _planPoint1.position.y, _planPoint2.position.x - _planPoint1.position.x);

        // ROS_INFO_STREAM_COND_NAMED(_debug, "cqu_recovery_globalplancb", "cqu_recovery_globalplancb: _globalKValue:" <<_globalKValue <<" degree: "<<atan(_globalKValue)/3.1415926*180);
    }

    double cqu_recovery::NormalizeDegree(double degree, double minDegree, double maxDegree)
    {
        if(degree>= minDegree && degree <= maxDegree)
            return degree;
        if(degree-minDegree>=0)
        {
            return minDegree + (fmodf64(degree-minDegree, maxDegree - minDegree));
        }
        else
        {
            return maxDegree + (fmodf64(degree-minDegree, maxDegree - minDegree));
        }
    }
    double cqu_recovery::Degree360(double degree)
    {
        if(degree<0)
        {
            return degree + 3.14159 * 2;
        }
        else
            return degree;
    }
}
