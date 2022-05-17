#!/usr/bin/evn python3
#Author: lnotspotl

import rospy
import numpy as np
import tf
import math

from std_msgs.msg import String

from . StateCommand import State, Command, BehaviorState
from . LieController import LieController
from . RestController import RestController
from . TrotGaitController import TrotGaitController
from . CrawlGaitController import CrawlGaitController
from . StandController import StandController

class Robot(object):
    def __init__(self, body, legs, imu):
        self.body = body
        self.legs = legs

        self.delta_x            = self.body[0] * 0.5
        self.delta_y            = self.body[1] * 0.5 + self.legs[1]
        self.x_shift_front      = 0.008
        self.x_shift_back       = -0.04
        self.default_height     = 0.15

        self.publisher_lcd_state    = rospy.Publisher("notspot_lcd/state", String, queue_size = 1)

        self.trotGaitController     = TrotGaitController(self.default_stance, stance_time = 0.26, swing_time = 0.24, time_step = 0.02, use_imu = imu)
        self.crawlGaitController    = CrawlGaitController(self.default_stance, stance_time = 0.55, swing_time = 0.45, time_step = 0.02)
        self.standController        = StandController(self.default_stance)

        self.lieController          = LieController(self.default_stance)
        self.restController         = RestController(self.default_stance)
        
        self.currentController      = self.restController
        self.state                  = State(self.default_height)
        self.state.foot_locations   = self.default_stance
        self.command                = Command(self.default_height)

    def change_controller(self):
        
        if self.command.trot_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.TROT
                self.currentController = self.trotGaitController
                self.currentController.pid_controller.reset()
                self.state.ticks = 0
            self.command.trot_event = False

        elif self.command.crawl_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.CRAWL
                self.currentController = self.crawlGaitController
                self.currentController.first_cycle = True;
                self.state.ticks = 0
            self.command.crawl_event = False

        elif self.command.stand_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.STAND
                self.currentController = self.standController
            self.command.stand_event = False

        elif self.command.rest_event:
            self.state.behavior_state = BehaviorState.REST
            self.currentController = self.restController
            self.currentController.pid_controller.reset()
            self.command.rest_event = False

        elif self.command.lie_event:
            self.state.behavior_state = BehaviorState.LIE
            self.currentController = self.lieController
            #self.currentController.pid_controller.reset()
            self.command.lie_event = False

    def joystick_command(self,msg):
        if msg.buttons[0]: # rest
            self.command.rest_event     = True
            self.command.trot_event     = False
            self.command.crawl_event    = False
            self.command.stand_event    = False
            self.command.lie_event      = False
            self.publisher_lcd_state.publish("rest")
            print("rest")

        elif msg.buttons[1]: # trot
            self.command.rest_event     = False
            self.command.trot_event     = True
            self.command.crawl_event    = False
            self.command.stand_event    = False
            self.command.lie_event      = False
            self.publisher_lcd_state.publish("trot")
            print("trot")

        elif msg.buttons[2]: # stand
            self.command.rest_event     = False
            self.command.trot_event     = False
            self.command.crawl_event    = False
            self.command.stand_event    = True
            self.command.lie_event      = False
            self.publisher_lcd_state.publish("stand")
            print("stand")

        elif msg.buttons[3]: # crawl
            self.command.rest_event     = False
            self.command.trot_event     = False
            self.command.crawl_event    = True
            self.command.stand_event    = False
            self.command.lie_event      = False
            self.publisher_lcd_state.publish("crawl")
            print("crawl")

        elif msg.buttons[10]: # lie
            self.command.rest_event     = False
            self.command.trot_event     = False
            self.command.crawl_event    = False
            self.command.stand_event    = False
            self.command.lie_event      = True
            self.publisher_lcd_state.publish("lie")
            print("lie")

        self.currentController.updateStateCommand(msg, self.state, self.command)

    def imu_orientation(self,msg):
        q = msg.orientation
        rpy_angles = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        self.state.imu_roll     = rpy_angles[0]
        self.state.imu_pitch    = rpy_angles[1]
        print('rpy_angles=', rpy_angles)

        # orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # linear_acceleration
        # angular_velocity
        #rpy_angles = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        #self.state.imu_roll = rpy_angles[0]
        #self.state.imu_pitch = rpy_angles[1]
        #self.state.imu_roll = q.x
        #self.state.imu_pitch = q.y

        # None
        # rpy_angles= (0.0005984700709411604, 0.005235679411140057, -0.0016590353742991784)
        # rpy_angles= (0.0005993580510755736, 0.00523535610401267, -0.0016596577658183008)
        # rpy_angles= (0.0006002456941323063, 0.005235032470537327, -0.0016602805879939802)

        # Right
        #rpy_angles= (0.06913512139945624, -0.014799836058811895, 0.013579090832045526)
        #rpy_angles= (0.06913565671324759, -0.014798509750779447, 0.013583737265416396)
        #rpy_angles= (0.06913619166810156, -0.014797184761385489, 0.013588383041694959)
        # Left
        #rpy_angles= (-0.18990422743629182, -0.008851821292948191, 0.009087492436355702)
        #rpy_angles= (-0.18990271149239166, -0.00884808095229375, 0.009089295303220233)
        #rpy_angles= (-0.1899011966053448, -0.008844342787192006, 0.009091097065168141)

        # Front Down
        #rpy_angles= (0.01350870715159488, 0.19861207481063764, 0.016167582627254852)
        #rpy_angles= (0.013508468161615706, 0.1986139798970741, 0.0161656979233706)
        #rpy_angles= (0.013508229284027083, 0.19861588391202964, 0.016163813093002723)

        # Rear Down
        #rpy_angles= (0.0026309815450587712, -0.17620387442458826, 0.011415160450937094)
        #rpy_angles= (0.002632293765070901, -0.17620502390516904, 0.011415803576638822)
        #rpy_angles= (0.0026336058783212914, -0.1762061733661644, 0.011416445656898035)

    def run(self):
        return self.currentController.run(self.state, self.command)

    @property
    def default_stance(self):
        return np.array([[self.delta_x + self.x_shift_front,self.delta_x + self.x_shift_front,-self.delta_x + self.x_shift_back,-self.delta_x + self.x_shift_back],
                         [-self.delta_y                    ,self.delta_y                     ,-self.delta_y                    , self.delta_y                    ],
                         [0                                ,0                                ,0                                ,0                                ]])

    @property
    def default_trotGait_stance(self):
        #                 FR,                              ,FL,                              ,RR                               ,RL
        return np.array([[self.delta_x + self.x_shift_front,self.delta_x + self.x_shift_front,-self.delta_x + self.x_shift_back,-self.delta_x + self.x_shift_back],
                         [0                                ,0                                ,0                                ,0                                ],
                         [-self.delta_y                    ,self.delta_y                     ,-self.delta_y                    , self.delta_y                    ]])