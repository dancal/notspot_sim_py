#!/usr/bin/env python3
#Author: dancal

from board import SCL, SDA

#from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit

from numpy import array_equal, sin, cos, pi
import numpy as np
import math
import time

# roslaunch notspot run_robot_real.launch

# roslaunch notspot run_robot_gazebo.launch
# roslaunch notspot run_robot_hardware.launch

# roslaunch notspot_mpu6050 mpu.launch
# roslaunch notspot_joystick ramped_joystick.launch
# roslaunch notspot_lcd lcd.launch

# apt-get install ros-noetic-joy
# apt-get install ros-noetic-imu-tools
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
    def __init__(self, posName, pca9685, servoPin, defAngle, direction, restPos):
        self.posName    = posName
        self.pca9685    = pca9685
        self.servoPin   = servoPin
        self.defAngle   = defAngle
        self.direction  = direction
        self.restPos    = restPos

        self.pca9685.servo[self.servoPin].actuation_range = 180
        self.pca9685.servo[self.servoPin].set_pulse_width_range(500, 2500)

    def deg2rad(self, deg):
        return deg * np.pi / 180.0

    def rad2deg(self, rad):
        deg = rad * 180.0 / np.pi
        #if deg > 90:
        #    deg = 90
        #elif deg < -90:
        #    deg = -90
        return deg

    def posChange(self, currentPos):
        if self.beforePos == currentPos:
            return False

        if int( math.floor(self.beforePos / 2) * 2) == currentPos:
            return False
        
        return True

    def moveAngle(self, angle):
        curPos                  = int(math.floor(((self.rad2deg(angle) * self.direction) + self.defAngle)))
        if self.posName == "RLL":
            if curPos < 50:
                curPos = 50
            if curPos > 110:
                curPos = 110
        if self.posName == "RRL":
            if curPos > 140:
                curPos = 140
        self.currentPos         = curPos + self.restPos
        if self.posChange(self.currentPos) == True:
            self.pca9685.servo[self.servoPin].angle = self.currentPos
            print(self.posName, 'curPos =', curPos, 'Angle = ', self.currentPos, 'beforePos =', self.beforePos, ' == servopin == ', self.servoPin, ', angle = ', angle, 'rad2deg = ', self.rad2deg(angle))

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
        print("hardware init")
        
        self.ServoKitB          = ServoKit(channels=16, address=0x40)
        self.ServoKitF          = ServoKit(channels=16, address=0x41)
        #self.ServoKitB          = None
        #self.ServoKitF          = None

        # FRONT
        self.servoMoters.append( ServoItem('FLS', self.ServoKitF, 0, 90,   1, 5))     # 0
        self.servoMoters.append( ServoItem('FLL', self.ServoKitF, 1, 60,   1, 50))     # 1
        self.servoMoters.append( ServoItem('FLF', self.ServoKitF, 2, 160,  1, 0))     # 2

        self.servoMoters.append( ServoItem('FRS', self.ServoKitF, 3, 90,   1, -10))     # 3
        self.servoMoters.append( ServoItem('FRL', self.ServoKitF, 4, 122, -1, -45))     # 4
        self.servoMoters.append( ServoItem('FRF', self.ServoKitF, 5, 22,  -1, -15))     # 5

        # REAR
        self.servoMoters.append( ServoItem('RLS', self.ServoKitB, 0, 90,  -1, -5))     # 6
        self.servoMoters.append( ServoItem('RLL', self.ServoKitB, 1, 48,   1, 60))     # 7
        self.servoMoters.append( ServoItem('RLF', self.ServoKitB, 2, 158,  1, 40))     # 8

        self.servoMoters.append( ServoItem('RRS', self.ServoKitB, 3, 90,  -1, -3))     # 9
        self.servoMoters.append( ServoItem('RRL', self.ServoKitB, 4, 134, -1, -75))     # 10
        self.servoMoters.append( ServoItem('RRF', self.ServoKitB, 5, 22,  -1, -30))     # 11

    def move(self, joint_angles, state):
        for i in range(len(self.servoMoters)):
            self.servoMoters[i].moveAngle(joint_angles[i])

        time.sleep(0.005)
        
if __name__ == "__main__":
    ps4 = ServoController()
