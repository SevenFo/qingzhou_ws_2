#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
from pickle import LIST
import cv2.aruco as aruco
import numpy as np
# import argparse
from _02GStreamer import *
import time
import rospy
# import matplotlib.pyplot as plt
# from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from _02CalculatePositon import *
from _03TrafficLight import *
from robot_vision.srv import app
def gstreamer_pipeline(
		# capture_width=3264, #原来的
		# capture_height=2464,
		capture_width=640,
		capture_height=480,
		display_width=640,
		display_height=480,
		# display_width=1920, #原来的
		# display_height=1080,
		framerate=40,
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
np.set_printoptions(suppress=True, precision=4)
controlFlag = 0
openColorDetector = 0
Frame = 1       # 从第1帧开始读取视频
starttime = 0
emdtime = 0
Video = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
Video.set(1, Frame)
# 红绿灯识别
def detector(Img):
    if ret == True:
        global Frame
        Frame = Frame + 1
        # print(Frame)
        # %% 由Marker计算小车的相对位置，同时得到marker图像区域
        # time1 = time.time()
            # print(Frame)
        CamPosition, MarkerROI = DealMarker(Img)  # CamPosition：(x,y,z)
        # time2 = time.time()
        # print("marker :{}".format(time2-time1))
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

# #####################提取ROI#############################
def region_of_interest(r_image):
    h = r_image.shape[0] #图片的高
    w = r_image.shape[1] #图片的宽
    gray = cv2.cvtColor(r_image, cv2.COLOR_BGR2GRAY) #转化为灰度图

    poly = np.array([
        [(0, h-100), (w, h-100), (w, h), (0, h)]  #将数组转化为矩阵，四个为长方形的四个顶点，从左上角开始顺时针,最下面竖着100个像素的图
    ])
    mask = np.zeros_like(gray) #输入为矩阵gray，输出为形状和gray一致的矩阵，其元素全部为黑色0
    cv2.fillPoly(mask, poly, 255)   #将mask的poly部分填充为白色255
    masked_image = cv2.bitwise_and(r_image,r_image, mask=mask)  #将r_image的mask区域提取出来给masked_image
    return masked_image

# ###############################蓝色分割############################
def color_seperate(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   #对目标图像进行色彩空间转换
    lower_hsv = np.array([100, 43, 46])          #设定蓝色下限
    upper_hsv = np.array([124, 255, 255])        #设定蓝色上限
    # lower_hsv = np.array([80, 60, 60])          #设定蓝色下限
    # upper_hsv = np.array([150, 180, 180])        #设定蓝色上限
    mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)  #依据设定的上下限对目标图像进行二值化转换，低于lower,高于upper都变成0，在中间为255
    dst = cv2.bitwise_and(image, image, mask=mask)    #将image的mask区域提取出来给dst,即找到蓝色区域并赋值给dst
    # cv2.imshow('blue', dst)  #查看蓝色区域的图像
    # cv2.waitKey(3)
    return dst

# ############################红色分割##############################
def color_seperate_1(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   #对目标图像进行色彩空间转换
    # lower_hsv = np.array([156, 43, 46])          #设定红色下限
    # upper_hsv = np.array([180, 255, 255])        #设定红色上限
    lower_hsv = np.array([0, 43, 46])          #设定红色下限
    upper_hsv = np.array([10, 255, 255])        #设定红色上限
    mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)  #依据设定的上下限对目标图像进行二值化转换，低于lower,高于upper都变成0，在中间为255
    dst = cv2.bitwise_and(image, image, mask=mask)    #将image的mask区域提取出来给dst,即找到红色区域并赋值给dst
    # cv2.imshow('red', dst)  #查看红色区域的图像
    # cv2.waitKey(3)
    return dst

# ################检测偏移的函数####################################
def pianyi_detect(img):
    pianyi=0
    pianyi_text=''


    ##############读取图像#########################
    (img_w, img_h) = img.shape[:2] #获取传入图片的长与宽   w是高 h是宽
    # print(img_h) #640-------这是宽
    lane_img=img.copy() #复制一份获取的图像给lane_img
    cropped_img=region_of_interest(lane_img) #对图像进行ROI的分割
    # cv2.imshow("ROI",cropped_img) #ROI区域的画面
    # cv2.waitKey(5)
    cropped_img_1=cropped_img.copy() #将ROI图像复制一份给cropped_img_1
    cropped_img = color_seperate(cropped_img) #将ROI图像中的蓝色部分提取出来
    gray_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY) #将提取的  ROI的蓝色部分转化为灰度图
    # cv2.imshow("gray",gray_img) #查看转化后的灰度图
    # cv2.waitKey(5)
    ret, img_thresh = cv2.threshold(gray_img,10, 255, cv2.THRESH_BINARY)  #大于10的地方就转化为白色255，返回两个值第一个是域值，第二个是二值图图像
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)) #返回一个4*4的椭圆形矩阵核，椭圆的地方是1，其他地方是0
    img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel) #开运算，运用核kernel先进行腐蚀，再进行膨胀
    img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel) #闭运算，运用核kernel先进行膨胀，再进行腐蚀
    # cv2.imshow("img_thresh",img_thresh) #开闭运算后的图像
    # cv2.waitKey(5)
    contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # print(len(contours))#findcontours返回两个值，一个是一组轮廓信息，还有一个是每条轮廓对应的属性
    # cv2.drawContours(img_thresh,contours,-1,(0,0,255),3) #将检测到的轮廓画上去 
    # cv2.imshow("img_thresh_2",img_thresh) #绘制轮廓后的图像
    # cv2.waitKey(5)
    ####1.如果检测到蓝色线或者蓝白线，进行判断###########
    nothing_point = 0 #无用的点
    if (len(contours) > 0):   # 如果检测到的轮廓数量大于0
        con_num = len(contours) #将轮廓的个数赋值给con_num
        contour1 = [] #将contour1 赋值为空列表，[]表示列表，列表是可变的序列
        for c1 in range(len(contours)): #遍历每一个轮廓
                for c2 in range(len(contours[c1])): #遍历每一个轮廓的轮廓上的点
                    contour1.append(contours[c1][c2]) #将每一个轮廓的每一个点都排列起来组成一个新列表，.append() 方法用于在列表末尾添加新的对象
        contour1 = np.array(contour1) #将组成的新列表转化为矩阵，方便下一步处理
        (x, y, w, h) = cv2.boundingRect(contour1) #用一个最小的矩形，把找到的所有的轮廓包起来，返回轮值x，y是矩阵左上点的坐标，w，h是矩阵的宽和高
        # print(h)
        # ####################1.1 同时检测到蓝白线和蓝线，删选出蓝白线，计算位置########################
        if w>img_h/2  and con_num >1 : #如果整体轮廓的宽度大于二分之图片的宽度，则说明同时检测到了蓝白线和蓝线
            mask=np.zeros_like(gray_img) #
            cv2.rectangle(mask, (x, y), (x + 250, y + h-3), (255, 255, 255), cv2.FILLED) #将mask的部分进行白色填充，参数为填充区域的左上角顶将gray_img转化为全是0的矩阵并赋值给mask即全黑点和右下角顶点
            # cv2.imshow('mask',mask) #进行填充后的mask的图像
            # cv2.waitKey(5)   
            temp_img=cv2.bitwise_and(img_thresh, img_thresh, mask=mask) #将优化后的二值图img_thresh中的mask区域提取出来给temp_img
            # cv2.imshow('temp_img',temp_img) #只剩下蓝白线的二值图图像
            # cv2.waitKey(5)   
            contours1, hierarchy = cv2.findContours(temp_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #对只剩下蓝白线的二值图图像进行轮廓检测
            if (len(contours1) > 0):
                contour2 = []
                for c1 in range(len(contours1)):
                    for c2 in range(len(contours1[c1])):
                        contour2.append(contours1[c1][c2])
                contour2 = np.array(contour2) #将蓝白线的轮廓信息存于contour2矩阵中
                (x1, y1, w1, h1) = cv2.boundingRect(contour2) #蓝白线的轮廓信息
                cv2.rectangle(img, (x1, y1), (x1 + w1, img_h), (255, 255, 255), 3)#白框-----同时检测到蓝线和蓝白线给蓝白线画白框——永远贴着底画矩形框
                pianyi=((x1+w1/2)-(img_h/2))*FOV_w/img_h #pianyi值为矩形方框的中线距离视野中央的实际距离
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

            #########################1.2 只检测到一条线，需要判断是蓝白线还是蓝线##############
        elif w<img_h/2  :
            # 如果是蓝白线
            if con_num>1: #轮廓数量大于1，就是有好几段蓝色
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 3) #黄框----------只检测到蓝白线并用黄框画出
                pianyi = ((x + w / 2) - (img_h / 2)) * FOV_w / img_h #pianyi值为矩形方框的中线距离视野中央的实际距离
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
                # #检测红线
                # cropped_img_1 = color_seperate_1(cropped_img_1)
                # gray_img = cv2.cvtColor(cropped_img_1, cv2.COLOR_BGR2GRAY)
                # # gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0, 0, cv2.BORDER_DEFAULT)
                # ret, img_thresh = cv2.threshold(gray_img, 10, 255, cv2.THRESH_BINARY)
                # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                # img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
                # img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel)
                # contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)    
                # if len(contours) >0 : #如果红色物体数量大于1，则说明就是在红线上
                #     contour2 = []
                #     for c1 in range(len(contours)):
                #         for c2 in range(len(contours[c1])):
                #             contour2.append(contours[c1][c2])
                #     contour2 = np.array(contour2)
                #     (x2, y2, w2, h2) = cv2.boundingRect(contour2)
                #     cv2.rectangle(img, (x2, y2), (x2 + w2, img_h), (255, 0, 255), 3) #红框
                #     pianyi = 60 - ((x2 + w2 / 2) - (img_h / 2)) * FOV_w / img_h #60是凑数
                #     pianyi_text='right'  
                    #竖直蓝线######
                # else: #看见一块蓝色并且真的是蓝线
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3) #蓝框-----------------只检测到蓝线并用蓝框画出
                pianyi = 60-((x + w / 2) - (img_h / 2)) * FOV_w / img_h #60凑数
                pianyi_text='left'
                    # if h <60 and w < 150 : #当只检测到中间最后一点蓝色时，判断为出去
                    #     print('99999')
                    #     return 999
        elif con_num == 1: #横向蓝线
            #检测红线
                cropped_img_1 = color_seperate_1(cropped_img_1)
                gray_img = cv2.cvtColor(cropped_img_1, cv2.COLOR_BGR2GRAY)
                # gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0, 0, cv2.BORDER_DEFAULT)
                ret, img_thresh = cv2.threshold(gray_img, 10, 255, cv2.THRESH_BINARY)
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
                img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel)
                contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)    
                if len(contours) >0 : #如果红色物体数量大于1，则说明就是在红线上
                    contour2 = []
                    for c1 in range(len(contours)):
                        for c2 in range(len(contours[c1])):
                            contour2.append(contours[c1][c2])
                    contour2 = np.array(contour2)
                    (x2, y2, w2, h2) = cv2.boundingRect(contour2)
                    cv2.rectangle(img, (x2, y2), (x2 + w2, img_h), (255, 0, 255), 3) #红框
                    pianyi = 13 - ((x2 + w2 / 2) - (img_h / 2)) * FOV_w / img_h #60是凑数
                    pianyi_text='right'  
                else: #看见一块蓝色并且真的是蓝线
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3) #蓝框-----------------只检测到蓝线并用蓝框画出
                    pianyi = 83-((x + w / 2) - (img_h / 2)) * FOV_w / img_h #平滑过渡
                    pianyi_text='left'
                    # if h <60 and w < 100: #当只检测到中间最后一点蓝色时，判断为出去
                    #     print('9999')
                    #     return 999
        else : #检测到了左下角的点了
            nothing_point = 1


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

    # cv2.imshow('final_img', img) #车道线全图
    # cv2.waitKey(1)
    # # 返回偏移量（cm）和偏移方向（左偏或者右偏）
    global pianyi_befor
    pianyi_now = abs(pianyi)
    # print("first {}".format(pianyi_now))

    # print("second {}".format(pianyi_now))   
    if pianyi_text == 'right' :
        pianyi_now = 0 - pianyi_now-7 #这个数要试
        # print("third {}".format(pianyi_now))   
    elif  pianyi_text == 'left' :
        pianyi_now =  pianyi_now
        # print("third {}".format(pianyi_now))   
    # print(nothing_point) #打印出有没有左下角点的干扰
    # if(abs(pianyi - pianyi_befor) > 30) or pianyi_befor == -pianyi_now  or nothing_point ==1: #去除剧烈跳变和检测到左下角点
    if pianyi_befor == -pianyi_now  or nothing_point ==1: #这一句如果加上防止突变有点危险
        pianyi_now = pianyi_befor           
    pianyi_befor = pianyi_now
    # print(pianyi_now) #暂时
    return pianyi_now
def iscontrolsubcb(data,arg):
    global controlFlag
    # if(data.data == 1):
    #     controlFlag = 1

def handle_app_req(req):
    global controlFlag
    global starttime
    global endtime
    global openColorDetector
    if(req.statue == 1):
        controlFlag = 1
        starttime = time.time()
        print("++++++open roadline*******")
    elif(req.statue == 0):
        controlFlag = 0
        print("++++++close roadline*******")
    elif(req.statue == 2):
        controlFlag = 2
        print("++++++close roadline*******")
    elif(req.statue == 3):
        print("++++++open color*******")
        openColorDetector = 1
    elif(req.statue == 4):
        openColorDetector = 0
        print("++++++close color*******")
        

    return 0

if __name__ == '__main__':
    # 输入图像设置
    data = Vector3()
    pianyibefore = 0
    pianyicount = 0
    pianyisamelist = [0,0,0,0,0,0,0,0]
    controlFlag = 10 #原来是10
    openColorDetector = 0 #原来是0
    global flag_traffic
    flag_traffic = 0
    iscontrolsub = rospy.Subscriber("/is_version_cont",Float32,iscontrolsubcb,1)
    s = rospy.Service('/vision_control', app, handle_app_req)
    cmdData = Twist()
    cmdpub = rospy.Publisher("/cmd_vel",Twist)
    rospy.init_node("detector")
    try:

        rospy.loginfo("detector node is started...")
        while not rospy.is_shutdown():
        # 初始化ros节点
            # time1 = time.time()
            ret, Img = Video.read()
            # time2 = time.time()
            # cv2.imshow('c',Img) #红绿灯输入全图
            # cv2.waitKey(5)
            # time2 = time.time()
            # print("get pic :{}".format(time2-time1))
            if(Img is None):
                print("img is none")
            else:
                # time1 = time.time()
                if(openColorDetector):
                    if (flag_traffic%7) == 0 :  #对帧率进行7分频，没到整数倍就不读进来
                    # time1 = time.time()
                        Img_1 = cv2.resize(Img,(1920,1080))
                        # time2 = time.time()
                        # print("resize :{}".format(time2-time1))
                        colortype = detector(Img_1)
                        print(colortype)
                    flag_traffic = flag_traffic + 1
                else:
                    colortype = -1
                # time2 = time.time()
                # print("traffic :{}".format(time2-time1))
                vision = rospy.Publisher('/pianyi', Vector3, queue_size=1)     
			# pub_position= rospy.Publisher('position',Vector3, queue_size=10)
			# rate = rospy.Rate(10)
			# while not rospy.is_shutdown():
                # print(colortype)
                if colortype == 0 : #红灯
                    # print('666')
                    data.x = 0
                    # vision.publish(data)
                elif colortype == 1 : #绿灯1
                    # print('777')
                    data.x = 1
                    # vision.publish(data)
                elif colortype == 2 : #绿灯2
                    # print('888')
                    data.x = 2
                    # vision.publish(data)                
                else:
                    # print("-999") #没灯
                    data.x = -999
                    # vision.publish(data)
                if(openColorDetector):
                    vision.publish(data)
                Img_2 = cv2.resize(Img,(640,480))
                Img_2 = Img_2[3: 475,3:635]  #切割掉左右下角干扰点
                Img_2 = cv2.resize(Img_2,(640,480))
                # cv2.imshow('c',Img_2) #车道线输入全图
                # cv2.waitKey(5)
                # time1 = time.time()
                if(controlFlag):
                    pianyi = pianyi_detect(Img_2)
                else:
                    pianyi = 0
                # time2 = time.time()
                # print("pianyi :{}".format(time2-time1))
                # print(pianyi)   
                # print(type(pianyi))
                # rospy.spinonce()

                # if(abs(pianyi - pianyibefore) >= abs(40)):
                #     print("****************out****************")
                if(controlFlag == 1 or False): #原来是false
                    # print(pianyi)
                    # print("vision controling...")
                    cmdData.linear.x = 1.2
                    cmdData.angular.z = (pianyi*1.6+ 3.489) /180.0*3.1415926 #新增加了data.y的系数和最后的常数项 k =0.80837
                    cmdpub.publish(cmdData)

                    # pianyicount = 0
                elif(controlFlag == 2):
                    print("****************stop****************")
                    cmdData.linear.x = 0.2
                    cmdData.angular.z = -0.2
                    cmdpub.publish(cmdData)
                # elif(pianyi == 0):
                #     pianyicount += 1
                #     if(pianyicount == 5):
                #         pianyicount =0
                #         print("****************out****************")

                # if(pianyi == pianyibefore):
                #     pianyicountfirstflag = True
                #     pianyicount2 += 1
                #     if(pianyicount2 == 5):
                #         pianyicount2 =0
                #         print("****************out****************") 
                pianyibefore = pianyi
                for index,value in enumerate(pianyisamelist):
                    if(index != len(pianyisamelist)-1):
                        pianyisamelist[index] = pianyisamelist[index+1]
                    else:
                        pianyisamelist[index] = pianyi
                if(len(set(pianyisamelist)) == 1 and (time.time()-starttime) > 6):
                    # print("****************out****************") #暂时
                    pianyi = 999
                    # print(time.time()-starttime)
                # print(pianyisamelist)
                if(controlFlag == 1):
                    data.y = pianyi
                    vision.publish(data)
                  
                    

    except rospy.ROSInterruptException:
        print('End')
        Video.release()    