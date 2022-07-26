#!/usr/bin/env python
# -*- coding: utf-8 -*-

import imp
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np

class image_converter:
    def __init__(self):    
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/yijiaxiaoche/camera1/image_raw", Image, self.callback)

    def callback(self,data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        rospy.loginfo("---------")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print (e)
            
        # 在opencv的显示窗口中绘制一个圆，作为标记
        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (60, 60), 30, (0,0,255), -1)
          
        bardet = cv2.barcode_BarcodeDetector()
        
        ok, decoded_info, decoded_type, corners = bardet.detectAndDecode(cv_image)
        
        cv2.imshow("cv_image",cv_image)  
        cv2.waitKey(3)
           
        # grayscaled = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
        # gradX = cv2.Sobel(grayscaled,ddepth = cv2.CV_32F,dx = 1,dy = 0,ksize = -1)
        # gradY = cv2.Sobel(grayscaled,ddepth = cv2.CV_32F,dx = 0,dy = 1,ksize = -1)
        # gradient = cv2.subtract(gradX, gradY)
        # gradient = cv2.convertScaleAbs(gradient)
        
        # blured = cv2.blur(gradient,(9,9))
        # cv2.imshow("Blur", blured)
        # # grey 
        
        # # th = cv2.adaptiveThreshold(grayscaled, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 131, 1)
            
        # retval,th = cv2.threshold(blured, 255, 255, cv2.THRESH_BINARY)
        
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(21,7))
        # closed = cv2.morphologyEx(th,cv2.MORPH_CLOSE,kernel)
        
        # closed = cv2.erode(closed,None,iterations = 4)
        # closed = cv2.dilate(closed,None,iterations = 4)
        
         
        # contours = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # contours = imutils.grab_contours(contours)
        # c = sorted(contours,key = cv2.contourArea,reverse=True)[0]
        # #为最大轮廓确定最小边框
        # rect = cv2.minAreaRect(c)
        # box = cv2.cv.BoxPoints(rect) if imutils.is_cv2() else cv2.boxPoints(rect)
        # box = np.int0(box)
        # #显示检测到的条形码
        # cv2.drawContours(cv_image,[box],-1,(0,255,0),3)
        # cv2.imshow('Image',cv_image)
        # cv2.waitKey(3)
        
        # # 读取图片并将其转化为灰度图片
        # image1 = image.copy()
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # cv2.imshow("gray",gray)
        # # 计算图像中x和y方向的Scharr梯度幅值表示
        # ddepth = cv2.cv.CV_32F if imutils.is_cv2() else cv2.CV_32F
        # gradX = cv2.Sobel(gray, ddepth=ddepth, dx=1, dy=0, ksize=-1)
        # gradY = cv2.Sobel(gray, ddepth=ddepth, dx=0, dy=1, ksize=-1)
        # # x方向的梯度减去y方向的梯度
        # gradient = cv2.subtract(gradX, gradY)
        # # 获取处理后的绝对值
        # gradient = cv2.convertScaleAbs(gradient)
        # cv2.imshow("gradient",gradient)
        # # 对处理后的结果进行模糊操作
        # blurred = cv2.blur(gradient, (9, 9))
        # cv2.imshow("blurred",blurred)
        # # 将其转化为二值图片
        # (_, thresh) = cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY)
        # cv2.imshow("thresh",thresh)
        # # 构建一个掩码并将其应用在二值图片中
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
        # closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        # cv2.imshow("closed1",closed)
        # # 执行多次膨胀和腐蚀操作
        # closed = cv2.erode(closed, None, iterations = 4)
        # closed = cv2.dilate(closed, None, iterations = 4)
        # cv2.imshow("closed2",closed)
        # # 在二值图像中寻找轮廓, 然后根据他们的区域大小对该轮廓进行排序，保留最大的一个轮廓
        # closed, cnts = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # cnts = imutils.grab_contours(cnts)
        # c = sorted(cnts, key = cv2.contourArea, reverse = True)[0]
        # # 计算最大的轮廓的最小外接矩形
        # rect = cv2.minAreaRect(c)
        # box = cv2.cv.BoxPoints(rect) if imutils.is_cv2() else cv2.boxPoints(rect)
        # box = np.int0(box)
        # # 绘制并显示结果
        # cv2.drawContours(image1, [box], -1, (0, 255, 0), 3)
        # cv2.imshow('Image',image1)
        # cv2.waitKey(3)

        # 再将opencv格式额数据转换成ros image格式的数据发布
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print (e)

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down cv_bridge_test node.")
        cv2.destroyAllWindows()
