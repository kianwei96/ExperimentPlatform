
import os
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.animation import FuncAnimation
import sys
from copy import copy
from math import cos, sin, atan, asin, pi
import math
import time
import os
import abc

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, String
from sensor_msgs.msg import Joy
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from map import PostProcessPose


class PostProcessPoseWrapper(PostProcessPose):
    def __init__(self):
        super(PostProcessPoseWrapper, self ).__init__(0.1, 0.2, 5)
        self.currentPosition = {"x": 0, "y": 0, "angle": 0}

    def callback(self, message):
        super(PostProcessPoseWrapper, self).callback(message)
        (roll, pitch, angle) = euler_from_quaternion([message.pose.pose.orientation.x,
                                                      message.pose.pose.orientation.y, 
                                                      message.pose.pose.orientation.z, 
                                                      message.pose.pose.orientation.w])
        self.currentPosition.angle = angle
        self.currentPosition.x     = message.pose.pose.position.x
        self.currentPosition.y     = message.pose.pose.position.y
        print(self.currentPosition)

    def get_current_position(self):
        return self.currentPosition


class BaseTestStrategy(object):
    def __init__(self, delay_resample_time):
        self.testResultPublisher = rospy.Publisher("test_result", String, queue_size=5)
        self.delay_resample_time = delay_resample_time

    @abc.abstractmethod
    def loop_looking_for_signal(self):
        pass

    def publish_result(self, currentPosition, targetPosition, is_delayed_result = False):
        if (is_delayed_result):
            is_delayed_result = 1
        else:
            is_delayed_result = 0
        message = "%s, %f, %f, %f, %f, %f, %f, %d" % (rospy.get_time(), currentPosition.x, currentPosition.y, currentPosition.angle, \
                                                        targetPosition.x, targetPosition.y, targetPosition.angle, is_delayed_result)
        self.testResultPublisher.publish(message)


class JoyTestStrategy(BaseTestStrategy):
    def __init__(self, delay_resample_time):
        super(JoyTestStrategy, self).__init__(delay_resample_time)

    def loop_looking_for_signal(self):
        rospy.Subscriber("joy", Joy, self.joy_callback)

    def joy_callback(self, message):
        pass


class KeyTestStrategy(BaseTestStrategy):
    def __init__(self, delay_resample_time):
        super(JoyTestStrategy, self).__init__(delay_resample_time)

    def loop_looking_for_signal(self):
        pass

class SamplingEvent(object):
    def __init__(self, testStrategy, num_points):
        rospy.Subscriber("joy", Joy, self.callback)
        self.publisher = rospy.Publisher("test_events", Int16, queue_size=1)
        self.num_points = num_points
        self.curr_point = 0

    def callback(self, message):
        """
        docstring
        """
        pass

if __name__ == "__main__":
    PostProcessPoseWrapper().callback("")  
