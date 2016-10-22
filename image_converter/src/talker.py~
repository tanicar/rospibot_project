#!/usr/bin/env python
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
from time import sleep

class image_converter:

    def __init__(self):
	rospy.init_node('image_converter', anonymous=True)
        image_pub = rospy.Publisher("image_topic",Image,queue_size=10)
	
	camera = PiCamera() # Raspberry pi camera

	rate = rospy.Rate(10) # 10hz publisher frequency

	while not rospy.is_shutdown():
	    rospy.loginfo("Publishing an image")
            bridge = CvBridge()
	    camera.capture("/home/pi/ros_catkin_ws/image_converter/temp/imgBuff.jpg") # Capture and save the temporary image
	    cv_img = cv2.imread("/home/pi/ros_catkin_ws/image_converter/temp/imgBuff.jpg",0) # Read the image with openCV
            msg_img = bridge.cv2_to_imgmsg(cv_img) # convert from openCV image to image message
            image_pub.publish(msg_img)
            rate.sleep()
        
    

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
