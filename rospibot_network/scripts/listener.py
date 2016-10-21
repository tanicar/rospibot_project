#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import time
import atexit

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " Movement direction: %s", data.data)
   
    input = data.data
    
    #setup motor object and its address
    mh = Adafruit_MotorHAT(addr=0x60)

    #at exit code, to auto-disable motor on shutdown
    def turnOffMotors():
        mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE) 
        atexit.register(turnOffMotors)

    #setup 2 motors
    m1 = mh.getMotor(1)
    m2 = mh.getMotor(2)

    #setup motors' speed
    speed = 150
    motorBalance = 16
    m1.setSpeed(speed + motorBalance) #left motor
    m2.setSpeed(speed) #right motor

    #loop to read input and eventually spin motors
    # while(True):   # no need it since the code only runs when input comes

    #move directions = "wasd"
    #move forward
    if(input == "w"):
        m1.run(Adafruit_MotorHAT.FORWARD)
	m2.run(Adafruit_MotorHAT.FORWARD)
    #move backward
    if(input == "s"):
        m1.run(Adafruit_MotorHAT.BACKWARD)
        m2.run(Adafruit_MotorHAT.BACKWARD)
    #turn right
    if(input == "d"):
        m1.run(Adafruit_MotorHAT.FORWARD)
        m2.run(Adafruit_MotorHAT.RELEASE)
    #turn left
    if(input == "a"):
        m1.run(Adafruit_MotorHAT.RELEASE)
        m2.run(Adafruit_MotorHAT.FORWARD)
    #stop
    if(input == "x"):
        m1.run(Adafruit_MotorHAT.RELEASE)
        m2.run(Adafruit_MotorHAT.RELEASE)
    #speed change
    if(input == "i"):
        if(speed < 230):
            speed = speed + 10
    if(input == "u"):
        if(speed > 10):
            speed = speed - 10
    m1.setSpeed(speed+motorBalance)
    m2.setSpeed(speed)
   

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
