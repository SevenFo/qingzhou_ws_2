#ifndef __UDP_SEND_H__
#define __UDP_SEND_H__
#include "ros/ros.h"
#include "sys/socket.h"
#include "sys/types.h"
#include "errno.h"
#include "netinet/in.h"
#include "arpa/inet.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sys/ioctl.h"
#include "std_msgs/Float32.h"
#include "fcntl.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

typedef struct ImgData
{
    char data[1024];
    int DataSize;
    int index;//第几帧
    int count;//总共多少
    bool begin;
    bool end;
} imgdata;


class UDP_Sender
{
private:
    const std::string ipaddr = "192.168.1.142";
    const std::string topicName = "/main_camera/image_raw";
    const short port = 5555;
    int shSrv;//socket handler
    ros::Subscriber imgSuber;
    ros::NodeHandle nh;

public:
    UDP_Sender(const ros::NodeHandle &nd);
    ~UDP_Sender();
    bool SendMsg(void * data,size_t dataSize);

    void imgCb(const sensor_msgs::Image::ConstPtr& msg);


    bool SendImg(char * begin,size_t dataSize);
};

#endif