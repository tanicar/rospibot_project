#!/usr/bin/env python

#
# This script allows to send strings over the rospibot_network topic
#
# script by Andrea Fioroni - andrifiore@gmail.com
# GitHub repo: https://github.com/tanicar/rospibot_project
#

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('rospibot_network', String, queue_size=10) # publish on rospibot_network topic
    rospy.init_node('rospibot_network_talker', anonymous=True) # init a rospibot_network_talker node
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown(): # loop until shutdown
        dir_str = raw_input("Choose direction: ")
        rospy.loginfo(dir_str)
        pub.publish(dir_str) # publish on the topic
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
