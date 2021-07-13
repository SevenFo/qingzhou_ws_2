#include "UDP_Send.h"

UDP_Sender *udpsender;

int main(int argc, char **  argv)
{
    ros::init(argc,argv,"UDP_Send_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(50);
    std::cout <<"hello"<<std::endl;
    udpsender = new UDP_Sender(nh);
    char tmp[205];
    while(ros::ok())
    {
        ros::spinOnce();
        //udpsender->SendMsg((void *)"hello world",sizeof("hello world"));
        rate.sleep();
    }
}
