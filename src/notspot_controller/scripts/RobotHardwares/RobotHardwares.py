#!/usr/bin/env python3
#Author: dancal

#from board import SCL, SDA
#from turtle import pos
#from adafruit_pca9685 import PCA9685
#from adafruit_motor import servoMoter

from adafruit_motor import servo
import Adafruit_PCA9685 as PCA9685
from adafruit_servokit import ServoKit

from numpy import array_equal, sin, cos, pi
import numpy as np

# roslaunch notspot run_robot_gazebo.launch
# roslaunch notspot run_robot_hardware.launch
# roslaunch notspot_joystick ramped_joystick.launch

# pip3 install adafruit-circuitpython-motor
# pip3 install adafruit-circuitpython-servokit
# pip install turtle==0.0.1

# FR, FL, RR, RL
class ServoController:
    i2c                 = None
    ServoKitF           = None
    ServoKitB           = None

    angles              = []
    anglesPre           = []

    # FR, FL, RR, RL
    servoMoters         = []

    def __init__(self):
        print("init")
        # FR, FL, RR, RL

        #self.pca9685_1 = PCA9685(address=0x40) # rear  [ 0, 1, 2, 3, 4, 5 ]
        #self.pca9685_2 = Adafruit_PCA9685.PCA9685(address=0x41) # front [ 0, 1, 2, 3, 4, 5 ]
        
        # self.pca9685_1 = PCA9685(self.i2c, address=self.pca9685_1_address, reference_clock_speed=self.pca9685_1_reference_clock_speed)
        # self.pca9685_1.frequency = self.pca9685_1_frequency

        # self.pca9685_2.channels[0]

        self.ServoKitB          = ServoKit(channels=6, address=0x40)
        self.ServoKitF          = ServoKit(channels=6, address=0x41)

        self.servoMoters        = []    # 2
        self.servoMoters.append( self.ServoKitF )
        self.servoMoters.append( self.ServoKitF )
        self.servoMoters.append( self.ServoKitF )
        self.servoMoters.append( self.ServoKitF )
        self.servoMoters.append( self.ServoKitF )
        self.servoMoters.append( self.ServoKitF )

        self.servoMoters.append( self.ServoKitB )
        self.servoMoters.append( self.ServoKitB )
        self.servoMoters.append( self.ServoKitB )
        self.servoMoters.append( self.ServoKitB )
        self.servoMoters.append( self.ServoKitB )
        self.servoMoters.append( self.ServoKitB )

    def _normalizePos(self, servoAngle):
        # FR, FL, RR, RL
        _servoAngle     = []
        for i in range(len(self.servoMoters)):
            val = 0

            # SHOULDER
            if i == 0 or i == 3 or i == 6 or i == 9:
                val = 90 - servoAngle[i]

            elif i == 1:
                # FRL
                val = 125 - (servoAngle[i] * 1)
            elif i == 2:
                # FRF
                val = 170 - (servoAngle[i] * -1) 

            elif i == 4:
                # FLL
                val = 55 - (servoAngle[i] * -1)
            elif i == 5:
                # FLF
                val = 10 + (servoAngle[i] * -1) 

            elif i == 7:
                # RRL
                val = 140 - (servoAngle[i] * 1)
            elif i == 8:
                # RRF
                val = 165 - (servoAngle[i] * -1) 

            elif i == 10:
                # RLL
                val = 40 - (servoAngle[i] * -1)
            elif i == 11:
                # RLF
                val = 15 + (servoAngle[i] * -1) 

            _servoAngle.append(val)

        return _servoAngle

    def _moveServo(self, servoAngle):
        
        for i in range(len(self.servoMoters)):
            servoKit    = self.servoMoters[i]
            servoKit.angle  = servoAngle[i]

        print('servoAngle=', servoAngle)

    def move(self, joint_angles, servoAngle):
        
        bEqual  = array_equal(servoAngle, self.anglesPre)
        if bEqual:
            return
        
        # FR, FL, RR, RL
        _servoAngle     = self._normalizePos(servoAngle)
        self._moveServo(_servoAngle)
        self.anglesPre  = servoAngle

if __name__ == "__main__":
    ps4 = ServoController()
