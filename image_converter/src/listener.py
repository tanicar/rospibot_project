#!/usr/bin/env python

#
# This script reads raw pictures from image_topic topic
# converts and displays them using openCV libraries
# this is designed to work on a remote computer
#
# script by Andrea Fioroni - andrifiore@gmail.com
# GitHub repo: https://github.com/tanicar/rospibot_project
#

import rospy
from std_msgs.msg import String
import time
from PIL import Image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def callback(data): # runs everytime an ImageMessage is uploaded on the topic
    rospy.loginfo(rospy.get_caller_id() + " Getting Camera Frames")

    bridge = CvBridge() # setup a CVbridge to convert from ImageMessage to CVimage
    img = cv2.resize(bridge.imgmsg_to_cv2(data), (600,400)) # convert the image and resize it
    cv2.imshow("Rospibot cam", img) # display the image
    cv2.waitKey(2) # keep displaying the image

def listener():

    rospy.init_node('image_converter_listener', anonymous=True) # create a image_converter_listener node
 
    rospy.Subscriber("image_topic",Image,callback,queue_size=1) # subscribe to image_topic

    rospy.spin() # loop until shutdown

if __name__ == '__main__':
    listener()



