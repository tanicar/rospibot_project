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
import serial

def talker():
    pub = rospy.Publisher('image_topic', Image, queue_size=1) # publish on image_topic topic
    rospy.init_node('image_converter_talker', anonymous=True) # start a image_converter_talker node

    serialAvailable = 0
    serialInput = "no value yet"
    brightnessSensor = 0

    try:
   	serialPort = serial.Serial('/dev/ttyUSB0', 9600) # start the serial port
    	serialAvailable = 1
	rospy.loginfo("Serial port communication started succesfully")
    except serial.serialutil.SerialException :
        rospy.loginfo("Serial port not available")

    # this function maps sensorValue, which is in inLower - inUpper range,
    # in the outLower - outUpper range assuming outLower > outUpper, since 
    # it converts an aquired luminosity value into a brightness gain
    # (the lower is the aquired value, the higher the output value)
    def translate(sensorValue, inLower, inUpper, outLower, outUpper):
	outRange = outUpper - outLower
	inRange = inUpper - inLower
	outValue = (outRange - (sensorValue * outRange / inRange))
	if outValue > 0:
	    return outValue
	return 0    

    camera = PiCamera() # Raspberry pi camera
    # setup some camera properties
    camera.resolution = (160,112)
    camera.framerate = 50
    camera.contrast = 8
    camera.saturation = 5
    camera.brightness = 30
    camera.sharpness = 0

    camera.start_preview() # start the preview
    time.sleep(2) # delay
    rawCapture = PiRGBArray(camera) # get a pixel array

    rate = rospy.Rate(30) # publisher frequency
    bridge = CvBridge() # setup CVbridge to convert from CVimage to ImageMessage

    while not rospy.is_shutdown(): # loop until shutdown
        
	if(serialAvailable == 1): # if available, read the value of the brightness sensor
	    try:
	    	serialInput = serialPort.readline() # read the analogic value of the brightneses sensor
           	serialPort.flushInput() # empty the serial port buffer
	 	brightnessSensor = translate(int(serialInput), 10, 100, 10, 40) # translate the luminosity value in a brightness gain
	    except : 
		brightnessSensor = 0 # no serial port communication available
		rospy.loginfo("Serial port exception")
	    camera.brightness = 20 + brightnessSensor # add a +20 cutoff to the brightness level
	    rospy.loginfo(" Sending an Image Message")
	    #rospy.loginfo(" Brightness level is %sBrightness gain is %d", serialInput, brightnessSensor)
 
	else:
	    rospy.loginfo(" Sending an Image Message")

	camera.capture(rawCapture, format='bgr',use_video_port=True) # capture a frame on rawCapture
        pub.publish(bridge.cv2_to_imgmsg(rawCapture.array, 'bgr8')) # pulish the image on the topic
        rawCapture.truncate(0) # truncate the raw capture and free the buffer
	rate.sleep

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
