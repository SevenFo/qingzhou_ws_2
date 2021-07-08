#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import cv_bridge
from pickle import LIST
import cv2.aruco as aruco
import numpy as np

# from imutils import paths
import argparse
from _02GStreamer import *
import time
import rospy
import matplotlib.pyplot as plt
# from std_msgs.msg import String
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from _02CalculatePositon import *
from _03TrafficLight import *
from robot_vision.srv import app
def gstreamer_pipeline(
		capture_width=3264,
		capture_height=2464,
		# capture_width=1280,
		# capture_height=720,
		# display_width=640,
		# display_height=480,
		display_width=1920,
		display_height=1080,
		framerate=4,
		flip_method=0,
):
	return (
			"nvarguscamerasrc ! "
			"video/x-raw(memory:NVMM), "
			"width=(int)%d, height=(int)%d, "
			"format=(string)NV12, framerate=(fraction)%d/1 ! "
			"nvvidconv flip-method=%d ! "
			"video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
			"videoconvert ! "
			"video/x-raw, format=(string)BGR ! appsink"
			% (
				capture_width,
				capture_height,
				framerate,
				flip_method,
				display_width,
				display_height,
			)
	)

# camera = CSICamera(width=224, height=224, capture_width=1080, capture_height=720, capture_fps=30)
np.set_printoptions(suppress=True, precision=4)

controlFlag = 0

# 载入参数，载入地图
Frame = 1       # 从第1000帧开始读取视频
#Video = cv2.VideoCapture('/home/ltz/test003.avi')
# Video = cv2.VideoCapture('/home/ltz/test004.mp4')
# Video = cv2.VideoCapture(0)
Video = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
#Video = cv2.VideoCapture('/home/ltz/test002.avi')
#Video = cv2.VideoCapture('/home/ltz/test001.avi')
#ImgTest = cv2.imread('/home/ltz/jtd/test 005.png')
Video.set(1, Frame)

# 红绿灯识别
def detector(Img):
	# while 1:
	# while not rospy.is_shutdown():
	# # while not rospy.is_shutdown():
	# 	# 逐帧读取
	# 	ret, Img = Video.read()
	#	Img  = cv2.resize(Img,(1920,1080),interpolation=cv2.INTER_AREA)
    if ret == True:
        global Frame
        Frame = Frame + 1
        # %% 由Marker计算小车的相对位置，同时得到marker图像区域
        #time1 = #time.#time()
        CamPosition, MarkerROI = DealMarker(Img)  # CamPosition：(x,y,z)
        #time2 = #time.#time()
        #print("marker :{}".format(time2-time1))
        a =  np.array([1])
        projection_matrix =  np.array([[3.04423340e+02, 0, 3.25529293e+02,0],
                                                                    [0, 4.99492126e+02, 2.95309865e+02,0],
                                                                    [0, 0, 1,0]])
        # CamPosition_1 =  np.append(CamPosition, a, axis=1)
        # CamPosition_T = CamPosition_1.reshape(CamPosition_1.shape[0], 1)
        # pos = np.matmul(projection_matrix, CamPosition_T)
        if CamPosition is not None:
            # CamPosition_1 =  np.hstack(CamPosition,a)
            # CamPosition_T = CamPosition_1.reshape(CamPosition_1.shape[0], 1)
            # pos = np.matmul(projection_matrix, CamPosition_T)
            tmpPosition = Vector3(CamPosition[0],CamPosition[1],0)
        else:
            # print ('position is none')
            tmpPosition = Vector3(0,0,0)
        # %% 实现交通灯颜色检测
        # time1 = time.time()
        LightColors = TrafficLight(MarkerROI, Img)  # LightColors：0-'Red', 1-'Yellow', 2-'Green'
        # time2 = time.time()
        # print("traffic light :{}".format(time2-time1))
        # print('Frame:', Frame)
        # print(CamPosition, LightColors)     # 输出小车位置和交通灯颜色
        # print(type(LightColors))
        if(len(LightColors) > 0):
            # print(LightColors)
            colortype = LightColors[0]
            return colortype
    else:
        print('no image')

flag = 0
pianyi_befor = 0
# WIDTH=3280
# HEIGHT=2464
# 视野宽(cm)
FOV_w=105

# 提取ROI
def region_of_interest(r_image):
    h = r_image.shape[0]
    w = r_image.shape[1]
    gray = cv2.cvtColor(r_image, cv2.COLOR_BGR2GRAY)

    poly = np.array([
        [(0, h-100), (w, h-100), (w, h), (0, h)]
    ])
    mask = np.zeros_like(gray)
    cv2.fillPoly(mask, poly, 255)   #fillPoly（） ： 多个多边形填充函数原型——cv2.fillPoly( image , [ 多边形顶点array1, 多边形顶点array2, … ] , RGB color)
    masked_image = cv2.bitwise_and(r_image,r_image, mask=mask)  #cv2.bitwise_and()是对二进制数据进行“与”操作，即对图像（灰度图像或彩色图像均可）每个像素值进行二进制“与”操作，1&1=1，1&0=0，0&1=0，0&0=0
    return masked_image

# 色彩分割
def color_seperate(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   #对目标图像进行色彩空间转换
    # lower_hsv = np.array([100, 43, 46])          #设定蓝色下限
    # upper_hsv = np.array([124, 255, 255])        #设定蓝色上限
    lower_hsv = np.array([80, 60, 60])          #设定蓝色下限
    upper_hsv = np.array([150, 180, 180])        #设定蓝色上限
    mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)  #依据设定的上下限对目标图像进行二值化转换
    dst = cv2.bitwise_and(image, image, mask=mask)    #将二值化图像与原图进行“与”操作；实际是提取前两个frame 的“与”结果，然后输出mask 为1的部分
                                                 #注意：括号中要写mask=xxx
    #dst[dst==0]=255
    #cv2.imshow('result', mask)                     #输出
    return dst

# 色彩分割
def color_seperate_1(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   #对目标图像进行色彩空间转换
    lower_hsv = np.array([156, 43, 46])          #设定红色下限
    upper_hsv = np.array([180, 255, 255])        #设定红色上限
    mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)  #依据设定的上下限对目标图像进行二值化转换
    dst = cv2.bitwise_and(image, image, mask=mask)    #将二值化图像与原图进行“与”操作；实际是提取前两个frame 的“与”结果，然后输出mask 为1的部分
                                                 #注意：括号中要写mask=xxx
    #dst[dst==0]=255
    #cv2.imshow('result', mask)                     #输出
    return dst

# 检测偏移的函数
def pianyi_detect(img):
    pianyi=0
    pianyi_text=''
    #读取图像
    (img_w, img_h) = img.shape[:2]
    lane_img=img.copy()
    cropped_img=region_of_interest(lane_img)
    # cv2.imshow("cpp",cropped_img)
    cropped_img_1=cropped_img.copy()

    cropped_img = color_seperate(cropped_img)

    gray_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
    #gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0, 0, cv2.BORDER_DEFAULT)
    # cv2.imshow("grw",gray_img)
    ret, img_thresh = cv2.threshold(gray_img,10, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
    img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel)
    contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.imshow("imm",img_thresh) #车道线黑白二值图
    # if(255 not in img_thresh[420] and 255 not in img_thresh[400]):
    #     # print("out")
    #     return 999
    # print(cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))
    #1.如果检测到蓝色线或者蓝白线，进行判断
    if (len(contours) > 0):
        con_num = len(contours)
        contour1 = []
        for c1 in range(len(contours)):
            for c2 in range(len(contours[c1])):
                contour1.append(contours[c1][c2])
                # cv2.imshow('c', c1)
        contour1 = np.array(contour1)

        # res = cv2.drawContours(img, contour1, -1, (0, 0, 255), 1)
        (x, y, w, h) = cv2.boundingRect(contour1)

        # 1.1 同时检测到蓝白线和蓝线，删选出蓝白线，计算位置
        if w>img_h/4:
            mask=np.zeros_like(gray_img)
            cv2.rectangle(mask, (x, y), (x + 250, y + h), (255, 255, 255), cv2.FILLED) #白框
            # cv2.imshow('ccccc',mask)
            temp_img=cv2.bitwise_and(img_thresh, img_thresh, mask=mask)
            # print(img_thresh)
            # a = 0
            # b = 0
            # for i in img_thresh:

            #     # print(i)
            #     for j in i:
            #         if(j>0):
            #             print(a,b)
            #         b= b+1
            #     a = a+1
            # time.sleep(5)
            # # if(img_thresh[])

            contours1, hierarchy = cv2.findContours(temp_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if (len(contours1) > 0):
                contour2 = []
                for c1 in range(len(contours1)):
                    for c2 in range(len(contours1[c1])):
                        contour2.append(contours1[c1][c2])
                contour2 = np.array(contour2)
                # cv2.imshow('c',contour2)
                # print(contour2)
                # res = cv2.drawContours(img, contour1, -1, (0, 0, 255), 1)

                (x1, y1, w1, h1) = cv2.boundingRect(contour2)
                cv2.rectangle(img, (x1, y1), (x1 + w1, img_h), (250, 250, 255), 3)
                if(255 not in img_thresh[420] and 255 not in img_thresh[400]):
                    # print("out")
                    return 999
                pianyi=((x1+w1/2)-(img_h/2))*FOV_w/img_h
                if pianyi>0:
                    #print('右偏')
                    pianyi_text='right'
                elif pianyi<0:
                    #print('左偏')
                    pianyi_text='left'
                else:
                    # print('左偏')
                    pianyi_text = 'stright'
                #print(pianyi)

        # 1.2 判断是只检测到一条线，需要判断是蓝白线还是蓝线
        else:
            # 如果是蓝白线
            if con_num>1:
                if(255 not in img_thresh[420] and 255 not in img_thresh[400]):
                # print("out")
                    return 999
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 3) #黄框
                pianyi = ((x + w / 2) - (img_h / 2)) * FOV_w / img_h
                if pianyi > 0:
                    #print('右偏')
                    pianyi_text='right'
                elif pianyi<0:
                    #print('左偏')
                    pianyi_text='left'
                else:
                    # print('左偏')
                    pianyi_text = 'stright'
                #print(pianyi)
            # 蓝线
            else:
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3) #蓝框
                pianyi = 40-((x + w / 2) - (img_h / 2)) * FOV_w / img_h
                pianyi_text='left'
                #print('左偏移')
                #print(pianyi)

    # 2.未检测到蓝线或者蓝白线，就检测红线
    else:
        cropped_img_1 = color_seperate_1(cropped_img_1)
        gray_img = cv2.cvtColor(cropped_img_1, cv2.COLOR_BGR2GRAY)
        # gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0, 0, cv2.BORDER_DEFAULT)
        ret, img_thresh = cv2.threshold(gray_img, 10, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
        img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel)
        contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if (len(contours) > 0):
            contour2 = []
            for c1 in range(len(contours)):
                for c2 in range(len(contours[c1])):
                    contour2.append(contours[c1][c2])
            contour2 = np.array(contour2)
            # res = cv2.drawContours(img, contour1, -1, (0, 0, 255), 1)
            (x2, y2, w2, h2) = cv2.boundingRect(contour2)
            cv2.rectangle(img, (x2, y2), (x2 + w2, img_h), (255, 0, 255), 3) #红框

            pianyi = 40 - ((x2 + w2 / 2) - (img_h / 2)) * FOV_w / img_h
            pianyi_text='right'
            #print('右偏移')
            #print(pianyi)

    # cv2.imshow('res1', img) #车道线全图
    key = cv2.waitKey(5)
    # if key != -1:
    # #     # rospy.loginfo(" try to exit")
    #     pass
    # # 返回偏移量（cm）和偏移方向（左偏或者右偏）
    global pianyi_befor
    pianyi_now = abs(pianyi)
    # print("first {}".format(pianyi_now))
    if(abs(pianyi_now - pianyi_befor) > 30):
        pianyi_now = pianyi_befor
    # print("second {}".format(pianyi_now))   
    if pianyi_text == 'right' :
        pianyi_now = 0 - pianyi_now
        # print("third {}".format(pianyi_now))   
    elif  pianyi_text == 'left' :
        pianyi_now =  pianyi_now
        # print("third {}".format(pianyi_now))   
    
        
    pianyi_befor = abs(pianyi_now)
    return pianyi_now
def iscontrolsubcb(data,arg):
    global controlFlag
    # if(data.data == 1):
    #     controlFlag = 1

def handle_app_req(req):
    global controlFlag
    if(req.statue == 1):
        controlFlag = 1
    elif(req.statue == 0):
        controlFlag = 0
    elif(req.statue == 2):
        controlFlag = 2

    return 0

if __name__ == '__main__':
    # 输入图像设置
    data = Vector3()
    iscontrolsub = rospy.Subscriber("/is_version_cont",Float32,iscontrolsubcb,1)
    s = rospy.Service('/vision_control', app, handle_app_req)
    cmdData = Twist()
    cmdpub = rospy.Publisher("/cmd_vel",Twist)
    # arg = argparse.ArgumentParser()
    # arg.add_argument("-i", "--images", required=True,
    #                  help="path to input directory of images")
    # args = vars(arg.parse_args())
    
    # cap.release() 
    # cv.destroyAllWindows() 
    # 循环读取文件夹下的图像进行处理
    # for imagePath in paths.list_images('/home/cquer/Documents/python/opencv/Image'): #args["images"]
    #     img = cv2.imread(imagePath)
    #     result = pianyi_detect(img)
    #     print(result)
    #     input('input any key to continue')
    try:

        rospy.loginfo("detector node is started...")
        while not rospy.is_shutdown():
        # 初始化ros节点
            rospy.init_node("detector")
            time1 = time.time()
            ret, Img = Video.read()
            time2 = time.time()
            print("get pic :{}".format(time2-time1))
            if(Img is None):
                print("img is none")
            else:
                # time1 = time.time()
                colortype = detector(Img)
                # time2 = time.time()
                # print("traffic :{}".format(time2-time1))
                vision = rospy.Publisher('/pianyi', Vector3, queue_size=1)     
			# pub_position= rospy.Publisher('position',Vector3, queue_size=10)
			# rate = rospy.Rate(10)
			# while not rospy.is_shutdown():
                # print(colortype)
                if colortype == 0 : #红灯
                    # print('666')
                    data.x = 666
                    vision.publish(data)
                elif colortype == 1 : #绿灯1
                    # print('777')
                    data.x = 777
                    vision.publish(data)
                elif colortype == 2 : #绿灯2
                    # print('888')
                    data.x = 888
                    vision.publish(data)                
                else:
                    # print("-999") #没灯
                    data.x = -999
                    vision.publish(data)
                Img_2 = cv2.resize(Img,(640,480))
                # cv2.imshow('c',Img_2) #红绿灯输入全图
                # cv2.waitKey(5)
                # time1 = time.time()
                pianyi = pianyi_detect(Img_2)
                # time2 = time.time()
                # print("pianyi :{}".format(time2-time1))
                data.y = pianyi
                vision.publish(data)
                print(pianyi)        
                # print(type(pianyi))
                # rospy.spinonce()

                
                if(controlFlag == 1):
                    print("vision controling...")
                    cmdData.linear.x = 0.3
                    cmdData.angular.z = data.y/180.0*3.1415926
                    cmdpub.publish(cmdData)
                elif(controlFlag == 2):
                    print("***************stop****************8")
                    cmdData.linear.x = 0
                    cmdData.angular.z = 0
                    cmdpub.publish(cmdData)
                    

    except rospy.ROSInterruptException:
        print('End')
        Video.release()    