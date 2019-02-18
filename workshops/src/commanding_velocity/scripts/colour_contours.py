# -*- coding: utf-8 -*-
"""
Created on Mon Feb 18 21:14:15 2019

@author: student
"""

import rospy
import cv2
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    def __init__(self):

        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
                                          Image, self.callback)
        self.pub = rospy.Publisher('/result_topic', String)
        
    def callback(self, data):
        cv2.namedWindow("Image window", 1)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        bgr_thresh = cv2.inRange(cv_image,
                                 numpy.array((200, 230, 230)),
                                 numpy.array((255, 255, 255)))

        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((90, 150, 0)),
                                 numpy.array((180, 255, 255)))

        lower_colour = numpy.array((110,50,50))
        upper_colour = numpy.array((130,255,255))
        mask = cv2.inRange(hsv_img, lower_colour, upper_colour)
        masked = cv2.bitwise_and(cv_image, cv_image, mask=mask)
       
        
    
        
        mean_blue =  str(numpy.mean(masked[:, :, 0]))
        mean_green = str(numpy.mean(masked[:, :, 1]))
        mean_red = str(numpy.mean(masked[:, :, 2]))
        self.pub.publish(mean_blue,mean_green,mean_red)

        _, bgr_contours, hierachy = cv2.findContours(
            bgr_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)

        _, hsv_contours, hierachy = cv2.findContours(
            hsv_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)
        for c in hsv_contours:
            a = cv2.contourArea(c)
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0))
        print '===='
        cv2.imshow("Image window", masked)
        cv2.waitKey(1)

    
image_converter()
rospy.init_node('image_converter', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()