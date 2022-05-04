#!/usr/bin/env python3
#Author: dancal

from board import SCL, SDA

#from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit

from numpy import array_equal, sin, cos, pi
import numpy as np
import math
# roslaunch notspot run_robot_gazebo.launch
# roslaunch notspot run_robot_hardware.launch
# roslaunch notspot_joystick ramped_joystick.launch

# apt-get install ros-noetick-joy
# pip3 install pygame
# pip3 install adafruit-circuitpython-motor
# pip3 install adafruit-circuitpython-servokit
# pip install turtle==0.0.1

class ServoItem:
    posName     = None
    pca9685     = None
    servoPin    = 0
    defAngle    = 0
    direction   = 1

    beforePos   = 0
    currentPos  = 0
    def __init__(self, posName, pca9685, servoPin, defAngle, direction):
        self.posName    = posName
        self.pca9685    = pca9685
        self.servoPin   = servoPin
        self.defAngle   = defAngle
        self.direction  = direction
        self.restPos    = 90

    def deg2rad(self, deg):
        return deg * np.pi / 180.0

    def rad2deg(self, rad):
        deg = abs(rad) * 180.0 / np.pi
        if deg > 90:
            deg = 90
        elif deg < -90:
            deg = -90
        return deg

    def posChange(self):
        if int( math.floor(self.beforePos / 2) * 2) == int( math.floor(self.currentPos / 2) * 2):
            return False
        return True

    def moveRest(self):
        self.currentPos = self.restPos  # int(math.floor(((self.restPos * self.direction) + self.defAngle)/2)*2)
        if self.posChange() == True:
            print(self.posName, ' == rest pos == ', self.currentPos)

        self.beforePos      =  self.currentPos

    def moveAngle(self, angle):

        self.currentPos          = int(math.floor(((self.rad2deg(angle) * self.direction) + self.defAngle)/2)*2)
        if self.posChange() == True:
            print(self.posName, ' == move pos == ', self.currentPos, ' == servopin == ', self.servoPin)
            self.pca9685.servo[self.servoPin].angle = self.currentPos

        self.beforePos      =  self.currentPos

# FR, FL, RR, RL
class ServoController:
    i2c                 = None
    ServoKitF           = None
    ServoKitB           = None

    servoDefAngle       = None

    servoAngle          = []
    servoAnglePre       = []

    # FR, FL, RR, RL
    servoMoters         = []

    def __init__(self):
        print("init")
        
        self.ServoKitB          = ServoKit(channels=16, address=0x40)
        self.ServoKitF          = ServoKit(channels=16, address=0x41)
        #self.ServoKitB          = None
        #self.ServoKitF          = None

        # notspot -> FR, FL, RR, RL
        # FRONT
        self.servoMoters.append( ServoItem('FRS', self.ServoKitF, 0, 90,  1))      # 0
        self.servoMoters.append( ServoItem('FRL', self.ServoKitF, 1, 126, -1))     # 1
        self.servoMoters.append( ServoItem('FRF', self.ServoKitF, 2, 169, -1))     # 2

        self.servoMoters.append( ServoItem('FLS', self.ServoKitF, 3, 90,   1))     # 3
        self.servoMoters.append( ServoItem('FLL', self.ServoKitF, 4, 55,   1))     # 4
        self.servoMoters.append( ServoItem('FLF', self.ServoKitF, 5, 169, -1))     # 5

        # REAR
        self.servoMoters.append( ServoItem('RRS', self.ServoKitB, 0, 90,   1))     # 6
        self.servoMoters.append( ServoItem('RRL', self.ServoKitB, 1, 139, -1))     # 7
        self.servoMoters.append( ServoItem('RRF', self.ServoKitB, 2, 167, -1))     # 8

        self.servoMoters.append( ServoItem('RLS', self.ServoKitB, 3, 90,   1))     # 9
        self.servoMoters.append( ServoItem('RLL', self.ServoKitB, 4, 42,   1))     # 10
        self.servoMoters.append( ServoItem('RLF', self.ServoKitB, 5, 167, -1))     # 11


    #def _moveServo(self, servoAngle):   
    #    for i in range(len(self.servoMoters)):
    #        servoKit    = self.servoMoters[i]
    #        servoKit.angle  = servoAngle[i]
    #    print('servoAngle=', servoAngle)

    def move(self, joint_angles, state):
        
        #print(state.wait_event)
        for i in range(len(self.servoMoters)):
            self.servoMoters[i].moveAngle(joint_angles[i])
            #if state.wait_event == False:
            #    self.servoMoters[i].moveAngle(joint_angles[i])
            #else:
            #    self.servoMoters[i].moveRest()

if __name__ == "__main__":
    ps4 = ServoController()
