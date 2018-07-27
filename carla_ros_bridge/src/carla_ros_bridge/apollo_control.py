"""
Class to handle pollo control input
"""

import math
import numpy as np
import rospy

from std_msgs.msg import String

class ApolloInputController(object):

    def __init__(self, topic_name='/apollo_control'):
        # current control co mmand
        self.cur_control = {
            'steer': 0.0,
            'throttle': 0.0,
            'brake': 0.0,
            'hand_brake': False,
            'reverse': False
        }

        self.cmd_vel_subscriber = rospy.Subscriber(
            '/apollo_control', String, self.set_control_cmd_callback)

    def set_control_cmd_callback(self, data):
        '''
            [0] steering
            [1] throttle
            [2] brake
            [3] hand_brake
            [4] reverse
        '''
        arr = data.data.split()

        self.cur_control['steer'] = float(arr[0])
        self.cur_control['throttle'] = float(arr[1])
        self.cur_control['brake'] = float(arr[2])
        self.cur_control['hand_brake'] = arr[3] == 'True'
        self.cur_control['reverse'] = arr[4] == 'True'
