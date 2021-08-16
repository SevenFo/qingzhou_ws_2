#ifndef CQU_RECOVERY_H
#define CQU_RECOVERY_H
#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <boost/thread.hpp>
#include <nav_msgs/Path.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include "nav_core/recovery_behavior.h"
#include "pluginlib/class_list_macros.h"
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Path.h"
#include <tf/tf.h>
#include <math.h>
namespace cqu_recovery_behavior
{
    class cqu_recovery : public nav_core::RecoveryBehavior
    {
    private:
        /* data */
        bool isInitialize;
    public:
        /**
         * @brief 初始化 recovery behavior
         * 
         * @param name 
         * @param tf A pointer to a transform listener
         * @param global_costmap A pointer to the global_costmap used by the navigation stack 
         * @param local_costmap A pointer to the local_costmap used by the navigation stack 
         */
        void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

        /**
         * @brief 执行恢复行为
         * 
         */
        void runBehavior();
        std::string _name;
        tf2_ros::Buffer* _tfPtr;
        costmap_2d::Costmap2DROS* _globalCostmapPtr;
        costmap_2d::Costmap2DROS* _localCostmapPtr;

        ros::NodeHandle _n;

        cqu_recovery()
        {
            isInitialize = false;
            
        }
        ~cqu_recovery()
        {
            delete _tfPtr;
            delete _globalCostmapPtr;
            delete _localCostmapPtr;
        }
    private:

        int _footlenth;//计算全局路径斜率时的步长
        double _globalKValue;
        double _localKValue;
        double _globalPlanDegree;
        double _localPlanDegree;


        bool isLocalPlanValid;
        bool isGlobalPlanValid;



        geometry_msgs::Pose _globalPlanPoint1;
        geometry_msgs::Pose _globalPlanPoint2;
        geometry_msgs::Pose _localPlanPoint1;
        geometry_msgs::Pose _localPlanPoint2;


        ros::Subscriber robotCurruentGoalSuber;
        ros::Subscriber globalPlanSuber;
        ros::Subscriber localPlanSuber;
        geometry_msgs::Pose _robotCurruentGoal;
        void RobotCurruentGoalCB(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void GlobalPlanCB(const nav_msgs::PathConstPtr &msg);
        void LocalPlanCB(const nav_msgs::PathConstPtr &msg);

        double CalculateKValue(geometry_msgs::Pose startp, geometry_msgs::Pose endp);
        double CalculateKValue(const geometry_msgs::Point startp, const geometry_msgs::Point endp);
        double NormalizeDegree(double degree, double minDegree=-3.14/2, double maxDegree=3.14/2);
        double KvalueToDegree(double Kvalue);
        double Degree360(double degree);
        bool _debug;
    };

} // namespace cqu_recovery_behavious
#endif