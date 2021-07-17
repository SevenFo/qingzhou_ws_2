#include "cqu_recovery/cqu_recovery.h"
#include "nav_core/recovery_behavior.h"
#include "pluginlib/class_list_macros.h"
#include <geometry_msgs/Twist.h>
#include <tf.h>
#include <math.h>
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
            this->robotCurruentGoalSuber = _n.subscribe<geometry_msgs::PoseStamped>("/move_base/curruent_goal", 1, cqu_recovery::RobotCurruentGoalCB, this);
            isInitialize = true;
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
        geometry_msgs::PoseStamped robotPose = geometry_msgs::PoseStamped();
        if(_globalCostmapPtr->getRobotPose(robotPose))
        {
            tf::Quaternion quat;
            tf::quaternionMsgToTF(robotPose.pose.orientation, quat);
            double robotYaw =  tf::getYaw(robotPose.pose.orientation);
            ROS_INFO_STREAM_NAMED("cqu_recovery:","K value:"<<std::atan(this->CalculateKValue(robotPose.pose,_robotCurruentGoal))<<" robotYaw:"<<robotYaw);
            if (std::atan(this->CalculateKValue(robotPose.pose,_robotCurruentGoal) >  robotYaw))
            {
                //
                ROS_INFO_STREAM_NAMED("cqu recovery", "cqu recovery: K_value > robotYaw");
            }
            else{
                //
                ROS_INFO_STREAM_NAMED("cqu recovery", "cqu recovery: K_value < robotYaw");
            }
        }
        ros::Duration sleepTime(0.5);
        ROS_INFO("action cqu recovery behavious, robot will run back for 0.5 s");
        ros::Publisher cmdVelPub;
        cmdVelPub = _n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        geometry_msgs::Twist tmpControl;
        tmpControl.linear.x = -0.5;
        tmpControl.angular.z = 0;
        cmdVelPub.publish(tmpControl);
        sleepTime.sleep();
        tmpControl.linear.x = 0;
        tmpControl.angular.z = 0;
        cmdVelPub.publish(tmpControl);
        ROS_INFO("cqu recovery behavious over!");
    }

double cqu_recovery::CalculateKValue(geometry_msgs::Pose startp,geometry_msgs::Pose endp)
{
    return (endp.position.y - startp.position.y) / (endp.position.x - startp.position.x);
}

    void cqu_recovery::RobotCurruentGoalCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        _robotCurruentGoal = msg.get()->pose;
    }
}