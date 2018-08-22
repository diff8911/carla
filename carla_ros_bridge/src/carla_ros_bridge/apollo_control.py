"""
Class to handle pollo control input
"""

import math
import numpy as np
import rospy

from std_msgs.msg import String

class ApolloInputController(object):

    def __init__(self, topic_name='/carla_control'):
        # current control command
        self.cur_control = {
            'steer': 0.0,
            'throttle': 0.0,
            'brake': 0.0,
            'hand_brake': False,
            'reverse': False
        }

        self.cmd_vel_subscriber = rospy.Subscriber(
            topic_name, String, self.set_control_cmd_callback)

    def set_control_cmd_callback(self, data):
        '''
            [0] steering
            [1] throttle
            [2] brake
        '''
        arr = data.data.split()

        self.cur_control['steer'] = -float(arr[0]) / 100.0
        self.cur_control['throttle'] = float(arr[1]) / 100.0 # TODO: Car is too slow.
        self.cur_control['brake'] = float(arr[2]) / 100.0