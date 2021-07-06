#include "TCP_Sender.h"

TCP_Sender *tcpsender;
bool acitonSetedGoal = false;//stop 0 start 1 getGood 2 throwGood 
rcm robotControlMsg;
void TimerCB(const ros::TimerEvent &e)
{
    tcpsender->SendRobotStatusInfo();
   // std::cout<<"log :clocked"<<std::endl;
}

// bool checkGoal(actionlib::SimpleClientGoalState actionResult)
// {
//     if(actionResult != actionlib::SimpleClientGoalState::SUCCEEDED)
//     {
//         ROS_INFO("exec goal failed stop run goal");
//         curruentGoal = 0;
//     }
//     else
//     {
//         ROS_INFO("move to next goal");
//         curruentGoal += 1;
//         if(curruentGoal == 4)
//             curruentGoal = 2;
//     }
// }

// bool runGoal(int goal = curruentGoal)
// {
//     switch((goal))
//     {
//         case 1:
//         {
//             ROS_INFO("Excuating goal");
//             tcpsender->startPoint.target_pose.header.stamp = ros::Time::now();
//             tcpsender->moveBaseActionClientPtr->sendGoal(tcpsender->startPoint);
//             ROS_INFO("waiting for action result");
//             tcpsender->moveBaseActionClientPtr->waitForResult();
//             checkGoal(tcpsender->moveBaseActionClientPtr->getState());
//             break;
//         }
//         case 2:
//         {
//             ROS_INFO("Excuating goal");
//             tcpsender->getGoodsPoint.target_pose.header.stamp = ros::Time::now();
//             tcpsender->moveBaseActionClientPtr->sendGoal(tcpsender->getGoodsPoint);
//             ROS_INFO("waiting for action result");
//             tcpsender->moveBaseActionClientPtr->waitForResult();
//             checkGoal(tcpsender->moveBaseActionClientPtr->getState());
//             break;
//         }
//         case 3:
//         {
//             ROS_INFO("Excuating goal");
//             tcpsender->throwGoodsPoint.target_pose.header.stamp = ros::Time::now();
//             tcpsender->moveBaseActionClientPtr->sendGoal(tcpsender->throwGoodsPoint);
//             ROS_INFO("waiting for action result");
//             tcpsender->moveBaseActionClientPtr->waitForResult();
//             checkGoal(tcpsender->moveBaseActionClientPtr->getState());
//             break;
//         }
//         default:
//         {
//             break;
//         }
//     }    
// }

int main(int argc, char **  argv)
{
    ros::init(argc,argv,"TCP_Send_node");
    ros::NodeHandle nh("~");
    ros::Timer timer = nh.createTimer(ros::Duration(0.5),TimerCB);
    ros::Rate rate(50);
    ros::Duration dsleep(5);
    tcpsender = new TCP_Sender(nh);
    if(tcpsender->SocketInit()) //暂时注释
    // if(true)
    {
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
                        ROS_INFO("set goal");
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
                        ROS_INFO("seted goal");
                        break;
                    }
                    case 0x03:
                    {
                        tcpsender->userPoint.target_pose.header.frame_id = "map";
                        tcpsender->userPoint.target_pose.pose.position.x = robotControlMsg.userPointX;
                        tcpsender->userPoint.target_pose.pose.position.y = robotControlMsg.userPointY;
                        tcpsender->userPoint.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotControlMsg.userPointZ);
                        ROS_INFO("Excuating user goal");
                        tcpsender->userPoint.target_pose.header.stamp = ros::Time::now();
                        tcpsender->moveBaseActionClientPtr->sendGoal(tcpsender->userPoint);
                        // ROS_INFO("waiting for action result");
                        // tcpsender->moveBaseActionClientPtr->waitForResult();
                        // if(tcpsender->moveBaseActionClientPtr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                        // {
                        //     ROS_INFO("reach user goal");
                        // }
                        // else
                        // {
                        //     ROS_INFO("unable to reach user goal");
                        // }
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
                        break;
                    }
                    default:
                    {
                        break;
                    }

                }
                //处理接受到的控制消息

            }


            // if(curruentGoal)//如果开始执行目标点
            // {

            // }
            // if(tcpsender->haveDetectedTfl)
            // {
            //     tcpsender->roadLineControl();
            // }
            //std::cout<<"goal state:"<<tcpsender->moveBaseActionClientPtr->getState().toString()<<std::endl;
            if(acitonSetedGoal)
                tcpsender->RunGoal();
            rate.sleep();

        }
        
    }
    else
    {
        std::cout<<"socket inited failed \n exit"<<std::endl;
        return -1;
    }
    
}
