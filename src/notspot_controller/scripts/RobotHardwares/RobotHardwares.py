#!/usr/bin/env python3
#Author: dancal

#from board import SCL, SDA
#from turtle import pos
#from adafruit_pca9685 import PCA9685
#from adafruit_motor import servoMoter

#from adafruit_motor import servo
#import Adafruit_PCA9685 as PCA9685
#from adafruit_servokit import ServoKit

from numpy import array_equal, sin, cos, pi
import numpy as np
import math
# roslaunch notspot run_robot_gazebo.launch
# roslaunch notspot run_robot_hardware.launch
# roslaunch notspot_joystick ramped_joystick.launch

# pip3 install adafruit-circuitpython-motor
# pip3 install adafruit-circuitpython-servokit
# pip install turtle==0.0.1

class ServoItem:
    pca9685Num  = None
    pca9685Pin  = 0

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
        # FR, FL, RR, RL

        #self.pca9685_1 = PCA9685(address=0x40) # rear  [ 0, 1, 2, 3, 4, 5 ]
        #self.pca9685_2 = Adafruit_PCA9685.PCA9685(address=0x41) # front [ 0, 1, 2, 3, 4, 5 ]
        
        # self.pca9685_1 = PCA9685(self.i2c, address=self.pca9685_1_address, reference_clock_speed=self.pca9685_1_reference_clock_speed)
        # self.pca9685_1.frequency = self.pca9685_1_frequency

        # self.pca9685_2.channels[0]

        #self.ServoKitB          = ServoKit(channels=6, address=0x40)
        #self.ServoKitF          = ServoKit(channels=6, address=0x41)
        self.ServoKitB          = None
        self.ServoKitF          = None

        #self.servoMoters['FRS'] = ServoItem(self.ServoKitF, 0)

        self.servoMoters.append( self.ServoKitF,  )
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

        self.servoDefAngle  = []
        self.servoDefAngle.append( 90 )
        self.servoDefAngle.append( 126 )
        self.servoDefAngle.append( 169 )

        self.servoDefAngle.append( 90 )
        self.servoDefAngle.append( 55 )
        self.servoDefAngle.append( 169 )

        self.servoDefAngle.append( 91 )
        self.servoDefAngle.append( 139 )
        self.servoDefAngle.append( 167 )

        self.servoDefAngle.append( 90 )
        self.servoDefAngle.append( 42 )
        self.servoDefAngle.append( 167 )

    def _spotStance(self, servoIdx):
        stance  = 1

        # Shoulder
        if (servoIdx % 3) == 2:
            stance  = -1

        if servoIdx == 1 or stance == 2:
            stance = -1

        if servoIdx == 7 or stance == 7:
            stance = -1

        return stance

    def rad2deg(self, rad):
        deg = abs(rad) * 180.0 / np.pi
        if deg > 90:
            deg = 90
        elif deg < -90:
            deg = -90
        return deg

    #def _moveServo(self, servoAngle):   
    #    for i in range(len(self.servoMoters)):
    #        servoKit    = self.servoMoters[i]
    #        servoKit.angle  = servoAngle[i]
    #    print('servoAngle=', servoAngle)

    def move(self, joint_angles, state):
        
        # servoAngle
        self.servoAngle = []
        for i in range(len(joint_angles)):
            stance      = self._spotStance(i)
            val         = (self.rad2deg(joint_angles[i]) * self._spotStance(i)) + self.servoDefAngle[i]
            
            self.servoAngle.append( int( math.floor(val / 2) * 2) )

        bEqual  = array_equal(self.servoAngle, self.servoAnglePre)
        if bEqual:
            return
        
        # FR, FL, RR, RL
        #_servoAngle     = self._normalizePos(self.servoAngle)
        #self._moveServo(_servoAngle)
        print(self.servoAngle, state.wait_event )
        self.servoAnglePre  = self.servoAngle

if __name__ == "__main__":
    ps4 = ServoController()
