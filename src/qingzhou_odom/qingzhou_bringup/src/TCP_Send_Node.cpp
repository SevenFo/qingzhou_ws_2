#include "TCP_Sender.h"

TCP_Sender *tcpsender;
bool acitonSetedGoal = false;//stop 0 start 1 getGood 2 throwGood 
rcm robotControlMsg = rcm();
void TimerCB(const ros::TimerEvent &e)
{
    tcpsender->SendRobotStatusInfo();
}

int main(int argc, char **  argv)
{
    ros::init(argc,argv,"TCP_Send_node");
    ros::NodeHandle nh("~");
    ros::Timer timer = nh.createTimer(ros::Duration(0.05),TimerCB);
    ros::Rate rate(50);
    ros::Duration dsleep(5);
    tcpsender = new TCP_Sender(nh);

    if(tcpsender->SocketInit()) //暂时注释
    // if(true)
    {
        tcpsender->WaitServices();
        tcpsender->ClearCostmapFirstly();
        timer.start(); //暂时注释

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
                        ROS_INFO_NAMED("TCP_Sender_Node","set goal");
                        tcpsender->UpdateRobotGoalList(robotControlMsg.goalList);
                        /***************v1*********************8
                        tcpsender->startPoint.target_pose.header.frame_id = "map";
                        tcpsender->getGoodsPoint.target_pose.header.frame_id = "map";
                        tcpsender->throwGoodsPoint.target_pose.header.frame_id = "map";
                        //tcpsender->startPoint.target_pose.header.stamp = ros::Time::now();
                        tcpsender->startPoint.target_pose.pose.position.x = robotControlMsg.startPointGoalX;
                        tcpsender->startPoint.target_pose.pose.position.y = robotControlMsg.startPointGoalY;
                        tcpsender->startPoint.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotControlMsg.startPointGoalZ);
                        tcpsender->getGoodsPoint.target_pose.pose.position.x = robotControlMsg.getGoodsPointX;
                        tcpsender->getGoodsPoint.target_pose.pose.position.y = robotControlMsg.getGoodsPointY;
                        tcpsender->getGoodsPoint.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotControlMsg.getGoodsPointZ);
                        tcpsender->throwGoodsPoint.target_pose.pose.position.x = robotControlMsg.throwGoodsPointX;
                        tcpsender->throwGoodsPoint.target_pose.pose.position.y = robotControlMsg.throwGoodsPointY;
                        tcpsender->throwGoodsPoint.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotControlMsg.throwGoodsPointZ);
                        ************************/
                        ROS_INFO_NAMED("TCP_Sender_Node","seted goal");
                        break;
                    }
                    case 0x03://改变目标
                    {
                        ROS_INFO_NAMED("TCP_Sender_Node","update goal");
                        tcpsender->UpdateRobotCurruentGoal(robotControlMsg.curruentGoal);
                        ROS_INFO_NAMED("TCP_Sender_Node","updated!");

                        /**********v1*******
                        tcpsender->ExecUserGoalAndUpdateLocation(robotControlMsg);
                        // tcpsender->userPoint.target_pose.header.frame_id = "map";
                        // tcpsender->userPoint.target_pose.pose.position.x = robotControlMsg.userPointX;
                        // tcpsender->userPoint.target_pose.pose.position.y = robotControlMsg.userPointY;
                        // tcpsender->userPoint.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotControlMsg.userPointZ);
                        // ROS_INFO("Excuating user goal");
                        // tcpsender->userPoint.target_pose.header.stamp = ros::Time::now();
                        // // tcpsender->robot
                        // tcpsender->moveBaseActionClientPtr->sendGoal(tcpsender->userPoint);
                        **********************/
                    }
                    case 0x04:
                    {
                        ROS_INFO("actiong setted goal");
                        if(acitonSetedGoal == false)
                        {
                            acitonSetedGoal = true;
                        }
                        else
                        {
                            acitonSetedGoal = false;
                        }
                        break;
                    }
                    case 0x05:
                    {
                        ROS_INFO_STREAM_NAMED("TCP_Sender_Node","Swtich auto goal control flag to"<<tcpsender->SwitchAutoGoalControlFlag());
                        break;
                    }
                    case 0x06:
                    {
                        if(tcpsender->SwitchVisionControl())
                            ROS_INFO_STREAM_NAMED("TCP_Sender_Node","Swtich vision control success");
                        else
                            ROS_INFO_STREAM_NAMED("TCP_Sender_Node","Swtich vision control failed");
                        break;
                    }
                    case 0x07:
                    {
                        if(tcpsender->SwitchTflControl())
                            ROS_INFO_STREAM_NAMED("TCP_Sender_Node","Swtich tfl control success");
                        else
                            ROS_INFO_STREAM_NAMED("TCP_Sender_Node","Swtich tfl control failed");
                        break;
                    }
                    case 0x08://update locaiton
                    {
                        ROS_INFO_STREAM_NAMED("TCP_Sender_Node","try to change robot location");
                        tcpsender->UpdateRobotLocation(robotControlMsg.location);
                        break;
                    }
                    default:
                    {
                        break;
                    }

                }
                //处理接受到的控制消息

            }
            if(acitonSetedGoal)
                tcpsender->RunGoal_v2();
            rate.sleep();

        }
        std::cout<<"quit TCP SEND NODE"<<std::endl;
        tcpsender->CloseSocket();
    }
    else
    {
        std::cout<<"socket inited failed \n exit"<<std::endl;
        return -1;
    }
    
}
