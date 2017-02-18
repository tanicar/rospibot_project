#!/usr/bin/env python

#
# This script gets raw pictures from raspberry camera and
# uploads them on the image_topic topic, after converting
# them with openCV libraries
#
# script by Andrea Fioroni - andrifiore@gmail.com
# GitHub repo: https://github.com/tanicar/rospibot_project
#

from __future__ import print_function
import roslib
roslib.load_manifest('image_converter')
import rospy
import sys
from std_msgs.msg import String
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from picamera import PiCamera
from picamera.array import PiRGBArray
import time

def talker():
    pub = rospy.Publisher('image_topic', Image, queue_size=1) # publish on image_topic topic
    rospy.init_node('image_converter_talker', anonymous=True) # start a image_converter_talker node
    
    camera = PiCamera() # Raspberry pi camera
    # setup some camera properties
    camera.resolution = (350,200)
    camera.framerate = 50
    camera.contrast = 10
    camera.saturation = 20
    camera.brightness = 60
    camera.sharpness = 0

    camera.start_preview() # start the preview
    time.sleep(2) # delay
    rawCapture = PiRGBArray(camera) # get a pixel array

    rate = rospy.Rate(30) # publisher frequency
    bridge = CvBridge() # setup CVbridge to convert from CVimage to ImageMessage

    while not rospy.is_shutdown(): # loop until shutdown
	camera.capture(rawCapture, format='bgr',use_video_port=True) # capture a frame on rawCapture, in BGR format	
        rospy.loginfo(" Sending an Image Message")
        pub.publish(bridge.cv2_to_imgmsg(rawCapture.array, 'bgr8')) # pulish the image on the topic
        rawCapture.truncate(0) # truncate the raw capture and free the buffer
	rate.sleep

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
