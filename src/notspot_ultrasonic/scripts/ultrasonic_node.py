#!/usr/bin/env python3

# sudo pip3 install adafruit-circuitpython-rgbled
# https://github.com/mmabey/CircuitPython_HCSR04
import RPi.GPIO as gpio
import time
import sys
import signal
import rospy
import board

import adafruit_rgbled
from hcsr04 import HCSR04

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class UltraSonic:

    sonar   = None
    trig = 27 # 7th
    echo = 17 # 6th
    
    rate    = 10
    distance_publisher  = None

    rgb_led     = None
    setdistance = 0 

    RED_LED     = board.D5
    GREEN_LED   = board.D6
    BLUE_LED    = board.D7
    def __init__(self, rate):

        rospy.init_node('ultrasonic_node', anonymous=True)
        rospy.loginfo(f"UltraSonic Sensor Init")

        self.sonar              = HCSR04(self.trig, self.echo)
        self.rgb_led            = adafruit_rgbled.RGBLED(self.RED_LED, self.GREEN_LED, self.BLUE_LED)

        self.distance_publisher = rospy.Publisher('notspot_ultrasonic/sonic_dist', Joy, queue_size=1)
        self.rate               = rospy.Rate(rate)

    def hex_to_rgb(self, hex):
        return tuple(int(hex[i:i+2], 16) for i in (0, 2, 4))

    def run(self):
        try:
            while not rospy.is_shutdown():

                distance = self.sonar.getTimeCM()

                #This if statement wont allow the object distance to exceed the inital set distance
                if 0 < distance < self.setdistance:
                    pass
                else:
                    distance = self.setdistance

                if distance <= 0:
                    self.rgb_led.color  = self.hex_to_rgb('FF0000')
                elif distance <= 1:
                    self.rgb_led.color  = self.hex_to_rgb('FF4500')
                elif distance <= 2:
                    self.rgb_led.color  = self.hex_to_rgb('FF8C00')
                elif distance <= 3:
                    self.rgb_led.color  = self.hex_to_rgb('FFA500')
                elif distance <= 4:
                    self.rgb_led.color  = self.hex_to_rgb('FFD700')
                elif distance <= 5:
                    self.rgb_led.color  = self.hex_to_rgb('B8860B')
                elif distance <= 6:
                    self.rgb_led.color  = self.hex_to_rgb('DAA520')

                elif distance <= 7:
                    self.rgb_led.color  = self.hex_to_rgb('006400')
                elif distance <= 8:
                    self.rgb_led.color  = self.hex_to_rgb('008000')
                elif distance <= 9:
                    self.rgb_led.color  = self.hex_to_rgb('228B22')
                elif distance >= 10:
                    self.rgb_led.color  = self.hex_to_rgb('00FF00')

                print('cm = ', distance)
                #print(self.sonar.dist_cm())
                # self.led.color    = self.color

                # joy = Joy()
                # joy.buttons = [1,0,0,0,0,0,0,0,0,0,0]       # rest
                # joy.axes    = [0.,0.,1.,0.,0.,1.,0.,0.]     # 
                # self.distance_publisher.publish(joy)
                self.rate.sleep()

        except KeyboardInterrupt:
            pass

        sonar.deinit()

if __name__ == "__main__":
    sonic = UltraSonic(rate = 60)
    sonic.run()