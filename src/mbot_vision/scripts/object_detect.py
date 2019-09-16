#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from math import *
from geometry_msgs.msg import Pose

BLUE_LOW   = 0
BLUE_HIGH  = 20
GREEN_LOW  = 20
GREEN_HIGH = 60
RED_LOW    = 80
RED_HIGH   = 150

class image_converter:
    def __init__(self):    
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("object_detect_image", Image, queue_size=1)
        self.target_pub = rospy.Publisher("object_detect_pose", Pose,  queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera_center/camera/image_raw", Image, self.callback)

    def callback(self,data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        # define the list of boundaries in BGR
        boundaries = [([BLUE_LOW, GREEN_LOW, RED_LOW], [BLUE_HIGH, GREEN_HIGH, RED_HIGH])]

        # loop over the boundaries
        # print(boundaries)
        for (lower, upper) in boundaries:
            # create NumPy arrays from the boundaries
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")

        # find the colors within the specified boundaries and apply the mask
        mask = cv2.inRange(cv_image, lower, upper)
        output = cv2.bitwise_and(cv_image, cv_image, mask = mask)

        cvImg = cv2.cvtColor(output, 6) #cv2.COLOR_BGR2GRAY
        npImg = np.asarray( cvImg )
        thresh = cv2.threshold(npImg, 1, 255, cv2.THRESH_BINARY)[1]

        # find contours in the thresholded image
        img, cnts, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        #cnts = cnts[0]

        # loop over the contours
        for c in cnts:
            # compute the center of the contour
            M = cv2.moments(c)

            if int(M["m00"]) not in range(20000, 250000):
                continue

            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            cv2.drawContours(cv_image, [c], -1, (0, 0, 255), 2)
            cv2.circle(cv_image, (cX, cY), 1, (0, 0, 255), -1)
            objPose = Pose()
            objPose.position.x = cX;
            objPose.position.y = cY;
            objPose.position.z = M["m00"];
            self.target_pub.publish(objPose)

        # 显示Opencv格式的图像
        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)

        # 再将opencv格式额数据转换成ros image格式的数据发布
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print e

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("object_detect")
        rospy.loginfo("Starting detect object")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down object_detect node."
        cv2.destroyAllWindows()
