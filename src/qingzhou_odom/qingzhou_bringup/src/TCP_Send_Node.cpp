#include "TCP_Sender.h"

TCP_Sender *tcpsender;

void TimerCB(const ros::TimerEvent &e)
{
    tcpsender->SendRobotStatusInfo();
    std::cout<<"log :clocked"<<std::endl;
}


int main(int argc, char **  argv)
{
    ros::init(argc,argv,"TCP_Send_node");
    ros::NodeHandle nh("~");
    ros::Timer timer = nh.createTimer(ros::Duration(0.5),TimerCB);
    ros::Rate rate(50);
    ros::Duration dsleep(5);
    tcpsender = new TCP_Sender(nh);
    if(tcpsender->SocketInit())
    {
        timer.start();
        rcm robotControlMsg;
        while(ros::ok())
        {
            ros::spinOnce();
            if(tcpsender->RetriveMsg(&robotControlMsg,sizeof(robotControlMsg)))
            {
                switch(robotControlMsg.controlFunctionSelector)
                {
                    case 0x01:
                    {
                        ROS_INFO("set speed");
                        geometry_msgs::Twist tmp;
                        tmp.linear.x = robotControlMsg.linearSpeed;
                        tmp.angular.z = robotControlMsg.angularSpeed;
                        tcpsender->cmdvelPuber.publish(tmp);
                        break;
                    }
                    case 0x02:
                    {
                        ROS_INFO("set goal");
                        tcpsender->startPoint.target_pose.header.frame_id = "map";
                        tcpsender->getGoodsPoint.target_pose.header.frame_id = "map";
                        tcpsender->throwGoodsPoint.target_pose.header.frame_id = "map";
                        //tcpsender->startPoint.target_pose.header.stamp = ros::Time::now();
                        tcpsender->startPoint.target_pose.pose.position.x = robotControlMsg.startPointGoalX;
                        tcpsender->startPoint.target_pose.pose.position.y = robotControlMsg.startPointGoalY;
                        tcpsender->getGoodsPoint.target_pose.pose.position.x = robotControlMsg.getGoodsPointX;
                        tcpsender->getGoodsPoint.target_pose.pose.position.y = robotControlMsg.getGoodsPointY;
                        tcpsender->throwGoodsPoint.target_pose.pose.position.x = robotControlMsg.throwGoodsPointX;
                        tcpsender->throwGoodsPoint.target_pose.pose.position.y = robotControlMsg.throwGoodsPointY;
                        ROS_INFO("sleep 5 second for excuating goal");
                        dsleep.sleep();
                        tcpsender->startPoint.target_pose.header.stamp = ros::Time::now();
                        tcpsender->moveBaseActionClientPtr->sendGoal(tcpsender->startPoint);
                        tcpsender->moveBaseActionClientPtr->waitForResult();
                        ROS_INFO("waiting for action result");
                        auto actionResult = tcpsender->moveBaseActionClientPtr->getState();
                        if(actionResult == actionlib::SimpleClientGoalState::SUCCEEDED)//success
                        {
                            ROS_INFO("Reach start point and move forward to get goods");
                            tcpsender->getGoodsPoint.target_pose.header.stamp = ros::Time::now();
                            tcpsender->moveBaseActionClientPtr->sendGoal(tcpsender->getGoodsPoint);
                            tcpsender->moveBaseActionClientPtr->waitForResult();
                            ROS_INFO("waiting for action result");
                            actionResult = tcpsender->moveBaseActionClientPtr->getState();
                            if(actionResult == actionlib::SimpleClientGoalState::SUCCEEDED)
                            {
                                ROS_INFO("Reach get goods point and move forward to throw goods");
                            }
                            else
                            {
                                ROS_INFO("Unable to reach get goods point");
                            }
                        }
                        else
                        {
                            ROS_INFO("Unable to reach start point");
                        }
                        break;
                    }
                    default:
                    {
                        break;
                    }

                }
                //处理接受到的控制消息

            }
            rate.sleep();
        }
    }
    else
    {
        std::cout<<"socket inited failed \n exit"<<std::endl;
        return -1;
    }
    
}
