#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import cv2
import numpy as np
import torch
from numpy import core
import glob
import logging
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist

from Timer import *
# from _00WarpMain import *
from _02PipeDatasetLoader import *
from _03Unet import *
from _04Loss import *
import time



def gstreamer_pipeline(
		capture_width=3264,
		capture_height=2464,
		# capture_width=640,
		# capture_height=480,        
		# capture_width=1280,
		# capture_height=720,
		# display_width=640,
		# display_height=480,
		display_width=1920,
		display_height=1080,
		framerate=3,
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
ImgPaths = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
H = np.array([[  -0.31606066,   -1.9361168,   631.35385969],
              [   0.01971609,    0.18515113, -173.82800143],
              [   0.00012182,   -0.00378804,    1.0       ]])

# 相机参数
Dist = np.array([-0.285977053720519, 0.074339925936036, -0.008145684407220, 0.0, 0.0], dtype=np.float32)
K = np.array([[308.7962753712953, 0, 325.1614349338094],
              [0, 309.9281771981168, 278.8305865054944],
              [0, 0, 1]], dtype=np.float32)
def line():
    rospy.init_node("detector")
    cmdData = Twist()
    cmdpub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
    # pianyi = line()
    rospy.loginfo("detector node is started...")
        # WarpedImg = cv2.resize(WarpedImg, (128, 128))
        # cv2.imshow('WarpedImg',WarpedImg) #查看变换后的图像
        # cv2.waitKey(1)
    print("loading net...")
    device = torch.device('cuda')
    torch.cuda.set_device(0)  # 选用GPU设备
    # 加载网络，图片单通道，分类为1。
    # net = UNet(n_channels=1, n_classes=1)
    net = UNet(in_channels=3, out_channels=1, init_features=4, WithActivateLast=True, ActivateFunLast=torch.sigmoid).to(device)
    # 将网络拷贝到deivce中
    net.to(device=device)
    # 加载模型参数
    net.load_state_dict(torch.load('/home/cquer/ROSApplications/qingzhou_ws_2/src/qingzhou_unet/scripts/0700_dict.pt', map_location='cuda'))

    # 测试模式
    net.eval()
    print("loaded net")
    while(not rospy.is_shutdown()):
        time1 = time.time()
        ret,Img = ImgPaths.read()
        Img = cv2.resize(Img, (640, 480))
        UndistImg = cv2.undistort(Img, K, Dist)
        WarpedImg = cv2.warpPerspective(UndistImg, H, (1000, 1000)) #变换后的图像
        # 读取图片
        b, g, r = cv2.split(WarpedImg)
        WarpedImg = cv2.merge([r, g, b])
        WarpedImg = Image.fromarray(WarpedImg)
        img_tensor = ValImgTransform(WarpedImg)
        # 转为tensor
        # img_tensor = torch.from_numpy(WarpedImg)
        # plt.imshow(np.array(img_tensor).transpose(1, 2, 0))
        # plt.show()
        # 将tensor拷贝到device中，只用cpu就是拷贝到cpu中，用cuda就是拷贝到cuda中。
        img_tensor = img_tensor.to(device=device, dtype=torch.float32)
        # img_tensor  = cv2.resize(img_tensor , (640, 480))
        # print(img_tensor.shape)
        img_tensor = img_tensor.reshape(1, 3, 128, 128)
        time2 = time.time()
        print("pred before: {}".format(time2-time1))
        # print(img_tensor.shape)
        # 预测
        time1 = time.time()
        pred = net(img_tensor)
        time2 = time.time()
        print("pred: {}".format(time2-time1))
        # print(pred.shape)
        # 提取结果
        time1  =time.time()
        pred = np.array(pred.data.cpu()[0])[0]
        # 处理结果
        # print(pred.shape)
        pred = (Normalization(pred) * 255).astype(np.uint8) #这个pred是灰度图 
        # pred = cv2.cvtColor(pred, cv2.COLOR_GRAY2RGB)#灰度图转RGB
        # pred =  cv2.adaptiveThreshold(pred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY, 25, 10)#二值化
        # cv2.imshow('pred',pred) #查看二值的图像#####################
        # cv2.waitKey(1)         
# ######################找线中心点###################################
#         num_lane_point = 10

#         # 读取所有图片路径
#         # tests_path = glob.glob('./images/*.jpg')
#         # for test_path in tests_path:
#         #     # 保存结果地址
#         #     save_res_path =  test_path.split('.')[0] + '_tracking.jpg'
#         #     img = cv2.imread(test_path)

#         ################## lane detection ##############################################
        num_lane_point = 10
        _, pred = cv2.threshold(pred, 128, 255, cv2.THRESH_BINARY)  # binaryzation

        height = 128
        width = 128
        half_width = 64
        center_num =0
        # img_out = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        img_out = pred
        # print(img_out.shape)
        # plt.imshow(img_out)
        # plt.show()
        out = 1
        for i in range(num_lane_point):         # each detected point on the lane
            detect_height = 150 - (height - 13 - i*10) #随便写的数
            detect_row = img_out[detect_height]
            line_line = np.where(detect_row == 255)  # extract zero pixel's index
            # print(line_line)
            if len(line_line[0]):                    # If this row has white pixels
                left_pos = np.min(line_line[0])
                right_pos = np.max(line_line[0])
            else:
                # left_pos = 0
                # right_pos = width - 1
                left_pos = 2126 #这一句和下一句为了凑999
                right_pos = 0
            
            center_left = (left_pos, detect_height)
            center_right = (right_pos, detect_height)
            center = (int((left_pos + right_pos)/2), detect_height)
            # if center != None : #计算中心点的实际个数
            #     center_num = center_num +1
            #第一级控制
            if center[1] > 30 and center[1] < 40 :  #center 每隔十个像素有一个，这个值要试！！！！
                control_num_1 = center[0] -64
                print('control_num_1 = %d' %control_num_1)
            # if center[1] > 40 and center[1] < 50 and center_num >1: #前面第二个中心点还有没有值
        time2 = time.time()
        print("pred after: {}".format(time2-time1))
        if control_num_1 == 999 :
            print('out of the line!!!')
        else :
            print('going')

        # 初始化ros节点

        if(control_num_1 != 999):
            print("vision controling...")
            cmdData.linear.x = 0.2
            cmdData.angular.z = -(control_num_1*1.1+0.9/180.0*3.1415926)
            cmdpub.publish(cmdData)
        else:
            cmdData.linear.x = 0.0

            cmdData.angular.z = 0.0
            cmdpub.publish(cmdData)

        
        
            

            # if left_pos != 0:   # draw the lane points on the binary image
            #     img_out = cv2.circle(img_out, center_left, 1, (0, 0, 255), thickness=1)
            # if right_pos != width - 1:
            #     img_out = cv2.circle(img_out, center_right, 1, (0, 0, 255), thickness=1)
            # if (left_pos != 0) and (right_pos != width - 1):
            #     img_out = cv2.circle(img_out, center, 1, (0, 255, 0), thickness=1)
        # cv2.imshow('center',img_out) #查看标注中心的图像#####################
        # cv2.waitKey(1) 
        # print(img_out.shape)-------------(128,128,3)

        ############像素检测法1######################################
        #####看白线在不在左边#############
        # left = 1
        # cropImg1 = pred[20:30,0:64]
        # # cv2.imshow('l',cropImg1)
        # # cv2.waitKey(1)
        # height = cropImg1.shape[0]
        # width = cropImg1.shape[1]
        # # colo =  cropImg1.shape[2]
        # for i in range(height):
        #     for j in range(width):
        #         if cropImg1[i,j] > 200 :
        #             left = left+1
        #             # print(left)

        # ######看白线在不在右边#############
        # right = 1
        # cropImg2 = pred[20:30,65:128]
        # # cv2.imshow('r',cropImg2)
        # # cv2.waitKey(1)
        # height = cropImg2.shape[0]
        # width = cropImg2.shape[1]
        # for i in range(height):
        #     for j in range(width):
        #         if cropImg2[i,j] >200 :
        #             right = right+1
        #             # print(right)
        # control_num = left -right
        # print(control_num)





if __name__ == "__main__":

    line()
