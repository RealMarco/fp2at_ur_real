#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 12 11:44:00 2023

@author: marco
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from mecheye_ros_interface.srv import CaptureColorMap

NUM=0
cv_bridge = CvBridge()

def capture_color_map():
    
    try:
        capture_color_map_service = rospy.ServiceProxy('/capture_color_map', CaptureColorMap)
        response = capture_color_map_service()
        if response.errorCode ==0:
        	rospy.loginfo(f'call for the service /capture_color_map successfully')
        else:
        	rospy.loginfo(f'Error: call for the service /capture_color_map UNsuccessfully')
    except rospy.ServiceException as e:
        print("Service call failed:", str(e))
		
def front_rgb_cb(msg):
	global NUM
	cv_image = cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
	cv2.imwrite(f'/home/marco/fp2a_ws/src/fp2a_ros/images/color_{NUM}.png', cv_image)
	NUM = NUM +1
	

if __name__ == '__main__':
    rospy.init_node('color_node')
    rospy.wait_for_service('/capture_color_map')
    front_rgb_sub = rospy.Subscriber('/mechmind/color_image', Image, front_rgb_cb)
	
    
    capture_color_map()
    capture_color_map()
	
    rospy.spin()
