#!/usr/bin/env python3

import time
import board
import sys
import signal
import rospy

import adafruit_rgbled
from std_msgs.msg import String

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class RgbSensor:
    # Pin the Red LED is connected to
    RED_LED     = board.D5

    # Pin the Green LED is connected to
    GREEN_LED   = board.D6

    # Pin the Blue LED is connected to
    BLUE_LED    = board.D7

    color       = (255, 255, 255)

    # Create the RGB LED object
    led         = None
    def __init__(self, rate):

        self.led    = adafruit_rgbled.RGBLED(self.RED_LED, self.GREEN_LED, self.BLUE_LED)
        time.sleep(0.5)

        self.rate   = rospy.Rate(rate)
        rospy.Subscriber("notspot_rgb/rgb_dist", String, self.callback_rgb)
        rospy.loginfo(f"Rgb Sensor Init")

    def hex_to_rgb(self, hex):
        return tuple(int(hex[i:i+2], 16) for i in (0, 2, 4))
        
    def callback_rgb(self, msg):
        self.color  = self.hex_to_rgb(msg.data)
        print('msg = ', msg, ', color = ', self.color)

    def run(self):

        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            self.led.color    = self.color
            self.rate.sleep()

if __name__ == "__main__":
    rgbLed = RgbSensor(rate = 60)
    rgbLed.run()