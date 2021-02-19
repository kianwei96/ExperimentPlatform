
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
        self.currentPosition = {"x": 0, "y": 0, "angle": 0, 'sampleTime': 0}

    def callback(self, message):
        super(PostProcessPoseWrapper, self).callback(message)
        (roll, pitch, angle) = euler_from_quaternion([message.pose.pose.orientation.x,
                                                      message.pose.pose.orientation.y, 
                                                      message.pose.pose.orientation.z, 
                                                      message.pose.pose.orientation.w])
        self.currentPosition["angle"] = angle
        self.currentPosition["x"]     = message.pose.pose.position.x
        self.currentPosition["y"]     = message.pose.pose.position.y
        # print(self.currentPosition)
        self.currentPosition["sampleTime"] = rospy.get_time()

    def get_current_position(self):
        return self.currentPosition


class BaseTestStrategy(object):
    def __init__(self, postProscessPoseObject):
        self.testResultPublisher = rospy.Publisher("test_result", String, queue_size=5)
        self.postProscessPoseObject = postProscessPoseObject

    @abc.abstractmethod
    def loop_looking_for_signal(self):
        pass

    @abc.abstractmethod
    def change_to_next_target(self, currentTargetPosition):
        pass

    def publish_result(self, currentPosition, targetPosition):
        message = "%f, %f, %f, %f, %f, %f, %f, %f" % (rospy.get_time(), currentPosition["sampleTime"],
                                                      currentPosition["x"], currentPosition["y"], currentPosition["angle"],
                                                      targetPosition["x"], targetPosition["y"], targetPosition["angle"])
        self.testResultPublisher.publish(message)


class JoyTestStrategy(BaseTestStrategy):
    def __init__(self, postProscessPoseObject):
        super(JoyTestStrategy, self).__init__(postProscessPoseObject)
        self.is_sampled = False
        self.targetPosition  = 0
        self.targetPositions =  [] #{"x": 0, "y": 0, "angle": 0}

    def loop_looking_for_signal(self):
        self.targetPositions = [{"x": x[1], "y": x[2], "angle": x[0]} for x in np.loadtxt("target.txt")]
        print(self.targetPositions)
        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)

    def joy_callback(self, message):
        if (int(message.buttons[0]) == 1): #capture a sample
            self.publish_result(self.postProscessPoseObject.currentPosition, self.targetPositions[self.targetPosition])
        if (int(message.buttons[1] == 1)):
            print("change target")
            self.change_to_next_target(self.targetPosition)

    def change_to_next_target(self, currentTargetPosition):
        self.targetPosition = int(input("Enter target index:"))
        # if (self.isTargetIndexIncrease):
        #     self.targetPosition = currentTargetPosition + 1
        # else: 
        #     self.targetPosition = currentTargetPosition - 1
        # if (self.targetPosition == 0 or self.targetPosition == self.numTargets-1):
        #     self.isTargetIndexIncrease = not self.isTargetIndexIncrease 

class KeyTestStrategy(BaseTestStrategy):
    def __init__(self, postProscessPoseObject):
        super(KeyTestStrategy, self).__init__(postProscessPoseObject)

    def loop_looking_for_signal(self):
        pass

    def change_to_next_target(self, currentTargetPosition):
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
    rospy.init_node('Post_Process', anonymous=True)
    test = JoyTestStrategy(PostProcessPoseWrapper())
    test.loop_looking_for_signal()
    rospy.spin()
