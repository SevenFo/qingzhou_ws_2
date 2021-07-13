#include "UDP_Send.h"

void UDP_Sender::imgCb(const sensor_msgs::Image::ConstPtr& msg)
{
  std::cout<<"log : get a image"<<std::endl;
    cv::Mat conversion_mat_;
  try
  {
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    conversion_mat_ = cv_ptr->image;
    
    // if (num_gridlines_ > 0)
    //   overlayGrid();
    auto imgSize = conversion_mat_.channels()*conversion_mat_.cols*conversion_mat_.rows;
    SendImg((char*)conversion_mat_.data,imgSize);
  }
  catch (cv_bridge::Exception& e)
  {
      std::cout<<"error "<<e.what();
/*     try
    {
      // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      if (msg->encoding == "CV_8UC3")
      {
        // assuming it is rgb
        conversion_mat_ = cv_ptr->image;
      } else if (msg->encoding == "8UC1") {
        // convert gray to rgb
        cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
      } else if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
        // scale / quantify
        double min = 0;
        double max = ui_.max_range_double_spin_box->value();
        if (msg->encoding == "16UC1") max *= 1000;
        if (ui_.dynamic_range_check_box->isChecked())
        {
          // dynamically adjust range based on min/max in image
          cv::minMaxLoc(cv_ptr->image, &min, &max);
          if (min == max) {
            // completely homogeneous images are displayed in gray
            min = 0;
            max = 2;
          }
        }
        cv::Mat img_scaled_8u;
        cv::Mat(cv_ptr->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));

        const auto color_scheme_index = ui_.color_scheme_combo_box->currentIndex();
        const auto color_scheme = ui_.color_scheme_combo_box->itemData(color_scheme_index).toInt();
        if (color_scheme == -1) {
          cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
        } else {
          cv::Mat img_color_scheme;
          cv::applyColorMap(img_scaled_8u, img_color_scheme, color_scheme);
          cv::cvtColor(img_color_scheme, conversion_mat_, CV_BGR2RGB);
        }
      } else {
        ROS_INFO("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
        return;
      }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_WARN("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
      return;
    } */
  }
}

UDP_Sender::UDP_Sender(const ros::NodeHandle &)
{
    this->nh = nh;
    imgSuber = nh.subscribe<sensor_msgs::Image>(topicName, 10, &UDP_Sender::imgCb,this);



    shSrv = socket(AF_INET,SOCK_DGRAM,0);
    if(shSrv == -1)
    {
        std::cout<<"error: socket init failed"<<std::endl;
    }
    else
    {
        std::cout<<"log: socket inited"<<std::endl;
    }
    


}

bool UDP_Sender::SendMsg(void * data,size_t dataSize)

{
    sockaddr_in addrSrv;

    addrSrv.sin_family = AF_INET;

    addrSrv.sin_port = htons(port);
   
    addrSrv.sin_addr.s_addr =  inet_addr("192.168.1.142");
    auto err = sendto(shSrv,data,dataSize,0,(sockaddr *)&(addrSrv),sizeof(addrSrv));
    if (err == -1)
        return false;
    else
    {
        std::cout<<"send "<<err<<"bytes"<<std::endl;
        return true;
    }
}

bool UDP_Sender::SendImg(char *begin,size_t imgSize)
{
  imgdata data;
  data.begin = false;
  data.end = false;
  int dataIndex = 0;//记录位置
  int count = 0;


  for(count = 0;count <= imgSize/1024;count++)
  {
    if(count == 0)
      data.begin = true;
    if(count == imgSize/1024)
      data.end == true;

    data.DataSize = 1024;
    data.index = count;

    for(int i = 0;i<data.DataSize;i++,dataIndex++)
    {
      std::cout <<"log : send data package :"<<count<<std::endl;
      if(dataIndex  >= imgSize)
      {
        data.data[i] == 0;
      }
      else
      {
        data.data[i] = *(begin+dataIndex);
      }
    }
    count++;
    
    SendMsg((void *)&data,sizeof(data));
  }
}