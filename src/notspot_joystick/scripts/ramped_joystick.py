#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# This file presents an interface for interacting with the Playstation 4 Controller
# in Python. Simply plug your PS4 controller into your computer using USB and run this
# script!
#
# NOTE: I assume in this script that the only joystick plugged in is the PS4 controller.
#       if this is not the case, you will need to change the class accordingly.
#
# Copyright © 2015 Clay L. McLeod <clay.l.mcleod@gmail.com>
#
# Distributed under terms of the MIT license.

import os
import pprint
import pygame

import rospy
from math import fabs
from numpy import array_equal
from time import sleep, time
from sensor_msgs.msg import Joy

class PS3Controller(object):
    """Class representing the PS4 controller. Pretty straightforward functionality."""

    controller = None
    axis_data = None
    button_data = None
    hat_data = None

    def init(self, rate):
        """Initialize the joystick components"""

        rospy.init_node("Joystick_ramped")
        #rospy.Subscriber("joy", Joy, self.callback)
        self.publisher = rospy.Publisher("notspot_joy/joy_ramped", Joy, queue_size = 10)
        self.rate = rospy.Rate(rate)

        self.speed_index = 0
        self.available_speeds = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5]

    def listen(self):
        
        pygame.init()
        pygame.joystick.init()

        clock           = pygame.time.Clock()

        joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
        for joy in joysticks:
            print(joy.get_name(), joy.get_id(), joy.get_guid(), joy.get_instance_id())

        joystick1           = pygame.joystick.get_count()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)

        self.axis_data      = [0.,0.,1.,0.,0.,1.,0.,0.]
        self.button_data    = [0,0,0,0,0,0,0,0,0,0,0]
        while not rospy.is_shutdown():
            
            evnet_changes   = False
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    evnet_changes               = True
                    self.axis_data[event.axis]  = (round(event.value,2) * -1) * self.available_speeds[self.speed_index]
                    #print(self.axis_data[event.axis])
                    #self.axis_data      = [0.,0.,1.,0.,0.,1.,0.,0.]
                    #self.axis_data[event.axis] = self.ramped_vel(self.axis_data_org[event.axis], event.value, self.last_send_time, t_now)
                elif event.type == pygame.JOYBUTTONUP:
                    if event.button == 5:        # return
                        self.speed_index += 1
                        if self.speed_index >= len(self.available_speeds):
                            self.speed_index = 0
                        rospy.loginfo(f"Joystick speed:{self.available_speeds[self.speed_index]}")
                    elif event.button == 9:      # list
                        self.speed_index -= 1
                        if self.speed_index < 0:
                            self.speed_index = 0
                        rospy.loginfo(f"Joystick speed:{self.available_speeds[self.speed_index]}")
                elif event.type == pygame.JOYBUTTONDOWN:
                    evnet_changes       = True
                    if event.button == 0:      # A
                        self.button_data    = [1,0,0,0,0,0,0,0,0,0,0]
                        rospy.loginfo("rest")
                    elif event.button == 1:      # B
                        self.button_data    = [0,1,0,0,0,0,0,0,0,0,0]                     
                        rospy.loginfo("trot")
                    elif event.button == 3:      # X
                        self.button_data    = [0,0,1,0,0,0,0,0,0,0,0]                   
                        rospy.loginfo("crawl")
                    elif event.button == 4:      # Y
                        self.button_data    = [0,0,0,1,0,0,0,0,0,0,0]                 
                        rospy.loginfo("stand")
                    else:
                        self.button_data    = [0,0,0,0,0,0,0,0,0,0,0]
                        #self.button_data[event.button] = 1
                elif event.type == pygame.JOYHATMOTION:
                    self.hat_data[event.hat] = event.value

            if evnet_changes == True:
                joy                 = Joy()
                joy.header.stamp    = rospy.Time.now()
                joy.axes            = self.axis_data
                joy.buttons         = self.button_data
                self.publisher.publish(joy)
                print(joy.buttons)

            self.rate.sleep()

if __name__ == "__main__":
    ps4 = PS3Controller()
    ps4.init(rate = 20)
    ps4.listen()