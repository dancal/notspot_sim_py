#!/usr/bin/env python3
#Author: lnotspotl

import rospy
import numpy as np
from RoboticsUtilities.Transformations import rotxyz
from . PIDController import PID_controller

class LieController(object):
    def __init__(self, default_stance):
        self.def_stance = default_stance

        # TODO: tune kp, ki and kd
        #                                     kp     ki    kd
        #self.pid_controller = PID_controller(1.75, 3.29, 0.0)
        #self.pid_controller = PID_controller(0.45, 1.29, 0.0)
        #self.pid_controller = PID_controller(0.45, 1.29, 0.0)
        #self.pid_controller.reset()
        
    def updateStateCommand(self, msg, state, command):
        # local body position
        print(msg.axes)
        state.body_local_position[0] = -0.2 * 0.04
        state.body_local_position[1] = 0
        state.body_local_position[2] = -0.07

    @property
    def default_stance(self):
        return self.def_stance

    def step(self, state, command):
        temp = self.default_stance
        temp[2] = [command.robot_height] * 4

        return temp

    def run(self, state, command):
        state.foot_locations = self.step(state, command)
        return state.foot_locations
