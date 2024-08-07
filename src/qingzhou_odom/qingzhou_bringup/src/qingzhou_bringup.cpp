#include "qingzhou_bringup/qingzhou_bringup.h"
// 基本上没有差别，甚至有些巧合的写得一样

const bool debug = false;
long long LeftticksPerMeter = 0;    
long long rightticksPerMeter = 0;   
long long LeftticksPer2PI = 0;      
long long rightticksPer2PI = 0;     

// 构造函数，初始化

actuator::actuator(ros::NodeHandle handle) 
{ 
   m_baudrate = 115200;             
   m_serialport = "/dev/stm32";   
   linearSpeed = 0;                 
   angularSpeed = 0;              
   batteryVoltage = 0;              
   ticksPerMeter = 0;               
   ticksPer2PI = 0;                 
   encoderLeft = 0;                 
   encoderRight = 0;               
   velDeltaTime = 0;                
   calibrate_lineSpeed = 0;         
   calibrate_angularSpeed = 0;      
    if(debug)
    std::cout<<"m_serial[prt:"<<m_serialport<<std::endl;
   memset(&moveBaseControl,0,sizeof(sMartcarControl));
    if(debug)
    std::cout<<"m_serial[prt:"<<m_serialport<<std::endl;
   handle.param("mcubaudrate",m_baudrate,m_baudrate);                                  
   handle.param("mcuserialport",m_serialport,std::string("/dev/stm32"));              
   handle.param("calibrate_lineSpeed",calibrate_lineSpeed,calibrate_lineSpeed);         
   handle.param("calibrate_angularSpeed",calibrate_angularSpeed,calibrate_angularSpeed);
   handle.param("ticksPerMeter",ticksPerMeter,ticksPerMeter);                           
   handle.param("ticksPer2PI",ticksPer2PI,ticksPer2PI);                                 

    try{ 

      std::cout<<"[qingzhou_actuator-->]"<<"Serial initialize start!"<<std::endl;              
      ser.setPort(m_serialport.c_str());                                               
      ser.setBaudrate(m_baudrate);                                                    
      serial::Timeout to = serial::Timeout::simpleTimeout(30);                      
      ser.setTimeout(to);                                                            
       if(debug)
        std::cout<<"[qingzhou_actuator-->][DEBUG]:try to open searal!"<<std::endl;                                                           
      ser.open();                                                                     
    }
    catch (serial::IOException& e){
      std::cout<<"[qingzhou_actuator-->]"<<"Unable to open port!"<<std::endl;                
    }
    if(ser.isOpen()){
      std::cout<<"[qingzhou_actuator-->]"<<"Serial initialize successfully!"<<std::endl;       
    }
    else{
      std::cout<<"[qingzhou_actuator-->]"<<"Serial port failed!"<<std::endl;                  
    } 
	
    sub_move_base = handle.subscribe("cmd_vel_filted",1,&actuator::callback_move_base,this);   

    pub_imu = handle.advertise<sensor_msgs::Imu>("raw", 2);	                                                 
    pub_mag = handle.advertise<sensor_msgs::MagneticField>("imu/mag", 5);                                    
    pub_odom = handle.advertise<nav_msgs::Odometry>("odom", 2);                                               
    pub_battery = handle.advertise<std_msgs::Float32>("battery",10);                                          
}

//析构函数
actuator::~actuator() 
{
     
}

void actuator::callback_move_base(const geometry_msgs::Twist::ConstPtr &msg) //对应cmd_vel话题，对应geometry_msgs/Twist消息
{
   memset(&moveBaseControl,0,sizeof(sMartcarControl));                       //清零movebase数据存储区
   
   float v = msg->linear.x;                                                  //move_base算得的线速度
   float w = msg->angular.z;                                                 //move_base算得的角速度

   moveBaseControl.TargetSpeed = v*32/0.43;                                  //计算目标线速度
   moveBaseControl.TargetAngle = round(atan(w*CARL/v)*57.3);                 //计算目标角度
   moveBaseControl.TargetAngle -= 0.018*57.3;
   //  float tmpang = msg->angular.z*1.5;
   //  moveBaseControl.TargetAngle = round(tmpang*(180.0/3.1415));
   //  moveBaseControl.TargetAngle+=60;                                          //stm32 program has subtract 60

   /*  printf("cmd_vel msg linx: %.2f,liny:%.2f,targetSpeed:%d,targetAngle:%d\n",msg->linear.x,msg->angular.z,                  
	       abs(moveBaseControl.TargetSpeed),abs(moveBaseControl.TargetAngle));	   
 */
}
  
void actuator::run()
{
    int run_rate = 50;              
    ros::Rate rate(run_rate);      

    double x = 0.0;                 //x坐标                       
    double y = 0.0;                 //y坐标
    double th = 0.0;

    ros::Time current_time, last_time;                    
	
    while(ros::ok()){
    ros::spinOnce();                                  
	current_time = ros::Time::now();                  //获得当前时间
	velDeltaTime = (current_time - last_time).toSec();//转换成秒
	last_time = ros::Time::now();                    
		
    recvCarInfoKernel();                              //接收stm32发来的数据
	pub_9250();                                       //发布imu数据
		
	
  int bit0_z = batteryVoltage & (255);
  int bit1_x = (unsigned char)((batteryVoltage) >> 8);
  currentBattery.data = bit0_z + ((float)bit1_x / 100.0f);
  // std::cout << "battary:" << (unsigned int)batteryVoltage << std::endl;
  // std::cout << "batteryVoltage: bits0:" << bit0_z << " bits1:" << bit1_x << std::endl;
  pub_battery.publish(currentBattery);             

    #if 1
	if(encoderLeft > 220 || encoderLeft < -220) encoderLeft = 0;
	if(encoderRight > 220 || encoderRight < -220) encoderRight = 0;
        //encoderLeft = -encoderLeft;
	encoderRight = -encoderRight;                          

	detEncode = (encoderLeft + encoderRight)/2;            
	detdistance = detEncode/ticksPerMeter;                 
	detth = (encoderRight - encoderLeft)*2*PI/ticksPer2PI; //计算当前角度,通过标定获得ticksPer2PI

	linearSpeed = detdistance/velDeltaTime;                
	angularSpeed = detth/velDeltaTime;                     
		
	if(detdistance != 0){
	    x += detdistance * cos(th);                        //x坐标
	    y += detdistance * sin(th);                        //y坐标
	}
	if(detth != 0){
	    th += detth;                                       //总角度
	}
    if(false)
        calibrate_angularSpeed = 1;
	if(calibrate_lineSpeed == 1){
		printf("x=%.2f,y=%.2f,th=%.2f,linearSpeed=%.2f,,detEncode=%.2f,LeftticksPerMeter = %lld,rightticksPerMeter = %lld,batteryVoltage = %d\n",x,y,th,linearSpeed,detEncode,LeftticksPerMeter,rightticksPerMeter,batteryVoltage);
	}
		
	//send command to stm32
	sendCarInfoKernel();                                                     
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th); 

	nav_msgs::Odometry odom;                               //创建nav_msgs::Odometry类型的消息odom
	odom.header.stamp = current_time;                     
	odom.header.frame_id = "odom";                         
 	odom.child_frame_id = "base_link";                     

	//set the position
	odom.pose.pose.position.x = x;                       
	odom.pose.pose.position.y = y;                        
	odom.pose.pose.position.z = 0.0;                       
	odom.pose.pose.orientation = odom_quat;               
	
	odom.twist.twist.linear.x = linearSpeed;               //线速度
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.linear.z = 0;
	odom.twist.twist.angular.x = 0;
	odom.twist.twist.angular.y = 0;
	odom.twist.twist.angular.z = angularSpeed;             //角速度
	if(encoderLeft == 0 && encoderRight == 0){
	    odom.pose.covariance = {1e-9, 0, 0, 0, 0, 0,       
				                0, 1e-3, 1e-9, 0, 0, 0,
				                0, 0, 1e6, 0, 0, 0,
				                0, 0, 0, 1e6, 0, 0,
				                0, 0, 0, 0, 1e6, 0,
				                0, 0, 0, 0, 0, 1e-9};

		odom.twist.covariance = {1e-9, 0, 0, 0, 0, 0,      
						        0, 1e-3, 1e-9, 0, 0, 0,
						        0, 0, 1e6, 0, 0, 0,
						        0, 0, 0, 1e6, 0, 0,
						        0, 0, 0, 0, 1e6, 0,
						        0, 0, 0, 0, 0, 1e-9};
		}
	else{
		odom.pose.covariance = {1e-3, 0, 0, 0, 0, 0,       
						        0, 1e-3, 0, 0, 0, 0,
						        0, 0, 1e6, 0, 0, 0,
						        0, 0, 0, 1e6, 0, 0,
						        0, 0, 0, 0, 1e6, 0,
						        0, 0, 0, 0, 0, 1e3};

		odom.twist.covariance = {1e-3, 0, 0, 0, 0, 0,      
						        0, 1e-3, 0, 0, 0, 0,
						        0, 0, 1e6, 0, 0, 0,
						        0, 0, 0, 1e6, 0, 0,
						        0, 0, 0, 0, 1e6, 0,
						        0, 0, 0, 0, 0, 1e3};
		}
	pub_odom.publish(odom);                                                            
	
#endif
    rate.sleep();
    }
}

//发送小车数据到下位机
void actuator::sendCarInfoKernel()
{
  if(moveBaseControl.TargetSpeed >= 0)
  {
    moveBaseControl.TargetShiftPosition = 1;

    if(moveBaseControl.TargetAngle >= 0)
      moveBaseControl.TargetAngleDir = 0x10;
    else
      moveBaseControl.TargetAngleDir = 0x20;
  }
  else
  {
    moveBaseControl.TargetShiftPosition = 2;
    if(moveBaseControl.TargetAngle <= 0)
      moveBaseControl.TargetAngleDir = 0x20;
    else
      moveBaseControl.TargetAngleDir = 0x10;
  }
  

    unsigned char buf[23] = {0};
    buf[0] = 0xa5;	                                        
    buf[1] = 0x5a;	                                        
    buf[2] = 0x06;	                                        

    buf[3] = (int)moveBaseControl.TargetAngleDir;	    //targetangleDirection 0-->go straight,0x10-->turn left,0x20-->turn right (not used)
    buf[4] = (int)abs(moveBaseControl.TargetAngle);	    //targetangle
    buf[5] = (int)abs(moveBaseControl.TargetSpeed);	    //targetSpeed
    buf[6] = (int)moveBaseControl.TargetModeSelect;	    //0-->person control,1-->auto control (not used)
    buf[7] = (int)moveBaseControl.TargetShiftPosition;  //targetshiftposition  0-->P stop;1-->R;2-->D. (not used)
  
    buf[8] = 8;		//heart beat                                        
    unsigned char sum = 0;
    for(int i = 2; i < 19; ++i)                             
        sum += buf[i];
    buf[9] = (unsigned char)(sum);                      
    size_t writesize = ser.write(buf,10);
    // debug = true;

    if (debug)
      std::cout << "debug: sended messages to actuator: speed: " << moveBaseControl.TargetSpeed << " angle:" << moveBaseControl.TargetAngle << " left or right:" << moveBaseControl.TargetAngleDir << " mode:" << moveBaseControl.TargetModeSelect << " shiftPosition:" << moveBaseControl.TargetShiftPosition << std::endl;
}

//接收下位机发送来的数据
void actuator::recvCarInfoKernel()
{
    std::string recvstr;                                             
    unsigned char tempdata,lenrecv;                                  
    unsigned char count,last_data,last_last_data,last_last_last_data;
    unsigned char str[100];                                          
    bool recvflag = false;                                          
    bool recvd_flag = false;                                         
    memset(&str,0,sizeof(str));                                      
    ros::Time begin_time = ros::Time::now();                         
    double clustering_time = 0;                                      

    while(1){
	    clustering_time = (ros::Time::now () - begin_time).toSec (); //计算时间差，转换成秒
	    if(clustering_time > 1){                                    
	        recvd_flag = false;                                      
	        break;                                                   
	    }
		
	    recvstr = ser.read(1);                                      		
	    if((int)recvstr.size() != 1)                                 
	        continue;                                               
		
	    tempdata = recvstr[0];                                       
	    if(last_last_last_data == 0xa5 && last_last_data == 0x5a){   
	        lenrecv = last_data;                                     
	        recvflag = true;                                         
	        count = 0;                                               
	    }
	    if(recvflag){                                               
	        str[count] = tempdata;                                   
	        count++;                                                
	        if(count == lenrecv){                                   
		        recvflag = false;                                    
		        recvd_flag = true;                                   
		    break;                                                  
	        }
	    }
	    last_last_last_data = last_last_data;                        
	    last_last_data = last_data;                                  
	    last_data = tempdata;                                        
    }

    if(recvd_flag){                                                  //数据解析，接收到的数据转存
	    memcpy(&encoderLeft,str,4);                                  
	    memcpy(&encoderRight,str+4,4);                              
	    memcpy(&batteryVoltage,str+8,4);             
         encoderRight = -encoderRight;                                     
 	
	    memcpy(&tempaccelX,str+12,2);                               
	    memcpy(&tempaccelY,str+14,2);                               
		memcpy(&tempaccelZ,str+16,2);                                
		
		memcpy(&tempgyroX,str+18,2);                                 
		memcpy(&tempgyroY,str+20,2);                                
		memcpy(&tempgyroZ,str+22,2);                                 
		
		memcpy(&tempmagX,str+24,2);                                 
		memcpy(&tempmagY,str+26,2);                                
		memcpy(&tempmagZ,str+28,2);                                

		accelX = (float)tempaccelX/2048*9.8;                         //线加速度处理	
		accelY = (float)tempaccelY/2048*9.8;
		accelZ = (float)tempaccelZ/2048*9.8;

		
		gyroX = (float)tempgyroX/16.4/57.3;                          //角速度处理
		gyroY = (float)tempgyroY/16.4/57.3;
		gyroZ = (float)tempgyroZ/16.4/57.3;
		
		magX = (float)tempmagX*0.14;                                 //磁力计处理
		magY = (float)tempmagY*0.14;
		magZ = (float)tempmagZ*0.14;

        if(encoderLeft > 220 || encoderLeft < -220) encoderLeft = 0; //判断编码器脉冲数是否在正确范围
        if(encoderRight > 220 || encoderRight < -220) encoderRight = 0;
        LeftticksPerMeter += encoderLeft;                            //获得左轮总脉冲数
        rightticksPerMeter += encoderRight;                          //获得右轮总脉冲数
    }
}

//发布imu函数
void actuator::pub_9250(){                         
    sensor_msgs::Imu imuMsg;                       
    sensor_msgs::MagneticField magMsg;           
	
    ros::Time current_time = ros::Time::now();    
	         
    imuMsg.header.stamp = current_time;            
    imuMsg.header.frame_id = "imu_link";          
    imuMsg.angular_velocity.x = gyroX;             
    imuMsg.angular_velocity.y = gyroY;             
    imuMsg.angular_velocity.z = gyroZ;             
    imuMsg.angular_velocity_covariance = {         
      0.04,0.0,0.0,
      0.0,0.04,0.0,
      0.0,0.0,0.04
    };
    
    imuMsg.linear_acceleration.x = accelX;        
    imuMsg.linear_acceleration.y = accelY;         
    imuMsg.linear_acceleration.z = accelZ;        
    imuMsg.linear_acceleration_covariance = {      
      0.04,0.0,0.0,
      0.0,0.04,0.0,
      0.0,0.0,0.04
    };
    pub_imu.publish(imuMsg);                       //发布imuMsg 
     
    magMsg.header.stamp = current_time;            
    magMsg.header.frame_id = "base_link";          
    magMsg.magnetic_field.x = magX;                
    magMsg.magnetic_field.y = magY;                
    magMsg.magnetic_field.z = magZ;                
    magMsg.magnetic_field_covariance = {           
      0.0,0.0,0.0,
      0.0,0.0,0.0,
      0.0,0.0,0.0
    };
    pub_mag.publish(magMsg);                       //发布magMsg
}
