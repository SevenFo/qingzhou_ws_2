#ifndef CQU_RECOVERY_H
#define CQU_RECOVERY_H
#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <boost/thread.hpp>
#include <dynamic_reconfigure/Reconfigure.h>
#include <move_base_msgs/MoveBaseGoal.h>

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
        ros::Subscriber robotCurruentGoalSuber;
        geometry_msgs::Pose _robotCurruentGoal;
        void RobotCurruentGoalCB(const geometry_msgs::PoseStamped::ConstPtr &msg);
        double CalculateKValue(geometry_msgs::Pose startp, geometry_msgs::Pose endp);
    };

} // namespace cqu_recovery_behavious
#endif