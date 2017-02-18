#!/usr/bin/env python

#
# This script allows control over a raspberry pi based robot
# reads strings from rospibot_network topic and decodes it 
# to control the motors using Adafruit libraries
#
# script by Andrea Fioroni - andrifiore@gmail.com
# GitHub repo: https://github.com/tanicar/rospibot_project
#

import rospy
import sys
from std_msgs.msg import String
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import time
import atexit
import RPi.GPIO as GPIO

class rospibot_network:

    def __init__(self):

	# semaphore variable
	self.redSemaphore = 0 # 1 = red semaphore is on

	# motor HAT setup
        self.mh = Adafruit_MotorHAT(addr=0x60) # setup Adafruit Motor HAT on 0x60 address
	
	# LED setup
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(18,GPIO.OUT)	
	GPIO.output(18,GPIO.HIGH)
	GPIO.setup(17,GPIO.OUT)
	GPIO.output(17,GPIO.HIGH)

        # at exit code, to auto-disable motor on shutdown
        def turnOffMotors():
            self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
            self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
	    GPIO.output(18,GPIO.LOW)
            atexit.register(turnOffMotors)    
	
	# setup 2 motors
        self.m1 = self.mh.getMotor(1) # left motor
        self.m2 = self.mh.getMotor(2) # right motor

        # setup motors' speed
        self.speed = 150 # default speed 
        self.motorBalance = 8 # speed cutoff (to balance the motors)
        self.m1.setSpeed(self.speed + self.motorBalance) # left motor
        self.m2.setSpeed(self.speed) # right motor
	
	rospy.Subscriber("rospibot_network", String, self.callback) # subscribe to rospibot_network topic

    def callback(self,data): # runs whenever any data is published on the topic
        rospy.loginfo(rospy.get_caller_id() + " Movement direction: %s", data.data)
   
        input = data.data # input received from rospibot_network (will always be a string)

        # move directions = "wasd"

	# red semaphore will disable any control on the robot
	# RRR = red semaphore - release the motors and set redSemaphore = 1
	if(input == "RRR"):
	    self.redSemaphore = 1
	    self.m1.run(Adafruit_MotorHAT.RELEASE)
	    self.m2.run(Adafruit_MotorHAT.RELEASE)
	# GGG = green semaphore
	if(self.redSemaphore == 1 and input == "GGG"):
	    self.redSemaphore = 0 
        # W - move forward
        if(self.redSemaphore == 0 and input == "w"):
            self.m1.run(Adafruit_MotorHAT.FORWARD)
	    self.m2.run(Adafruit_MotorHAT.FORWARD)
	    GPIO.output(17,GPIO.LOW)
        # S - move backward
        if(self.redSemaphore == 0 and input == "s"):
            self.m1.run(Adafruit_MotorHAT.BACKWARD)
            self.m2.run(Adafruit_MotorHAT.BACKWARD)
	    GPIO.output(17,GPIO.LOW)
        # D - turn right
        if(self.redSemaphore == 0 and input == "d"):
            self.m1.run(Adafruit_MotorHAT.FORWARD)
            self.m2.run(Adafruit_MotorHAT.RELEASE)
	    GPIO.output(17,GPIO.LOW)
        # A - turn left
        if(self.redSemaphore == 0 and input == "a"):
            self.m1.run(Adafruit_MotorHAT.RELEASE)
            self.m2.run(Adafruit_MotorHAT.FORWARD)
	    GPIO.output(17,GPIO.LOW)
        # X - stop
        if(input == "x"):
            self.m1.run(Adafruit_MotorHAT.RELEASE)
            self.m2.run(Adafruit_MotorHAT.RELEASE)
	    GPIO.output(17,GPIO.HIGH)
        #speed change
        if(input == "i"):
            if(self.speed < 230):
                self.speed = self.speed + 10
        if(input == "u"):
            if(self.speed > 10):
                self.speed = self.speed - 10
        self.m1.setSpeed(self.speed+self.motorBalance)
        self.m2.setSpeed(self.speed)
	# shutdown string
        if(input == "shutdown"):
	    self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
            self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
            GPIO.output(18,GPIO.LOW)
	    GPIO.output(17,GPIO.LOW)

def main(args):
    rospi_net = rospibot_network()
    rospy.init_node('rospibot_network_listener', anonymous=True) # create a rospibot_network_listener node
    try:
	rospy.spin() # loop until shutdown
    except KeyboardInterrupt:
	print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
