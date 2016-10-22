#!/usr/bin/env python
from __future__ import print_function
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

    def __init__(self):
      self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)

      self.bridge = CvBridge()
      self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)

    def callback(self,data):
      try:
        cv_image = self.bridge.imgmsg_to_cv2(data)
	rospy.loginfo(rospy.get_caller_id() + " Received an image")
	cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)	

      except CvBridgeError as e:
        print(e)

        

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





