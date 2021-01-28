#!/usr/bin/env python


import os
import cv2
import csv
import numpy as np
import sys
from math import cos, sin, atan, asin, pi
import gui

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, String
from sensor_msgs.msg import Joy
import time
import os
import pickle
import json

class LaserSubs(object):
    laser_ranges = 0

    def __init__(self):
        scan = LaserScan()
        self.init_laser_range()
        rospy.Subscriber('/base_scan', LaserScan, self.LaserData)

    def LaserData(self,msg):
        self.laser_ranges = msg.ranges

    def init_laser_range(self):
        self.laser_ranges = None
        for i in range(3):
            # while self.laser_ranges is None:
            if self.laser_ranges is None:
                try:
                    laser_data = rospy.wait_for_message('/base_scan', LaserScan, timeout=5)
                    self.laser_ranges = laser_data.ranges
                    time.sleep(0.05)
                except:
                    print('Waiting for base_scan to be ready')
                    time.sleep(0.05)


class LidarProcessor(object):
    dist_slow = 0
    dist_stop = 0

    flag_f = 0
    flag_l = 0
    flag_r = 0
    flag_fl = 0
    flag_fr = 0
    flag_old = [flag_f,flag_l,flag_r,flag_fl,flag_fr]
    flag_new = [flag_f,flag_l,flag_r,flag_fl,flag_fr]

    pub = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=1)

    def __init__(self,settings):
        self.laser_subs_object = LaserSubs()
        self.init_distance(settings)

    def init_distance(self, settings):
        self.dist_stop = settings.get("platform_stop_dist")
        self.dist_slow  = settings.get("platform_clear_dist")
        # self.dist_slow = 200
        # self.dist_stop = 100
        print("dist stop:", self.dist_stop)
        print("dist slow:", self.dist_slow)

    def update_distance(self):
        # print (self.laser_subs_object)
        dist_r = self.calc_avg(self.laser_subs_object.laser_ranges[0:193])
        dist_fr = self.calc_avg(self.laser_subs_object.laser_ranges[193:386])
        dist_f = self.calc_avg(self.laser_subs_object.laser_ranges[386:579])
        dist_fl = self.calc_avg(self.laser_subs_object.laser_ranges[579:772])
        dist_l = self.calc_avg(self.laser_subs_object.laser_ranges[772:963])

        self.save_data((dist_f))

        self.update_flags(dist_f,dist_l,dist_r,dist_fl,dist_fr)

    def update_speed(self):
        if self.flag_old != self.flag_new:
            if self.flag_f == 3 | self.flag_fl == 3 | self.flag_fr == 3:
                print("force stop")
                twist = Twist() # force stop if needed
                self.pub.publish(twist)
        pass

    def update_flags(self,dist_f,dist_l,dist_r,dist_fl,dist_fr):
        self.flag_old = [self.flag_f, self.flag_l, self.flag_r, self.flag_fl, self.flag_fr]
        self.flag_f = self.update_flag(dist_f)
        self.flag_l = self.update_flag(dist_l)
        self.flag_r = self.update_flag(dist_r)
        self.flag_fl = self.update_flag(dist_fl)
        self.flag_fr = self.update_flag(dist_fr)
        self.flag_new = [self.flag_f, self.flag_l, self.flag_r, self.flag_fl, self.flag_fr]

    def update_flag(self, dist):
        if dist >= self.dist_slow:
            flag = 1
        elif dist <=self.dist_stop:
            flag = 3
        else:
            flag = 2
        # flag = 2
        return flag

    @staticmethod
    def save_data(data):
        append_file = open("/home/sinapse/Desktop/MonkeyGUI-master/RewardData/LIDARData.txt", "a")
        # Save in a 2 by 2 array or it will print all integers vertically
        np.savetxt(append_file, [data], fmt="%f", delimiter=",")
        append_file.close()

    @staticmethod
    def calc_avg(values):
        return sum(values) / len(values)


class JoystickProcessor(object):
    speed_slow = 0
    speed_fast = 0

    pub = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=1)

    def __init__(self, settings, lidar):
        self.sub = rospy.Subscriber("joy", Joy, self.callback)
        self.lidar = lidar
        self.init_speed(settings)

    def init_speed(self,settings):
        self.speed_fast = settings.get("platform_normalSpeed")
        self.speed_slow = settings.get("platform_slowDownSpeed")
        # self.speed_slow = 0
        # self.speed_fast = 0
        print("speed slow:", self.speed_slow)
        print("speed fast:", self.speed_fast)

    def callback(self, data):
        global clamp
        print('front_flag:', self.lidar.flag_f,
              'left_flag:', self.lidar.flag_l,
              'right_flag:', self.lidar.flag_r)
        print('clamp: ' + str(clamp))
        self.move(data)

    def move(self, data):
        twist = Twist()

        # Data.axis[1] = joystick moving front and back, data.axis[0] = joystick moving left and right
        speed = data.axes[1] * 2
        if speed > 0:  # Joystick is indicating to move forward
            twist =  self.move_forward(self.lidar.flag_f, speed, twist)
        if speed < 0:  # Joystick is indicating to move in a reverse direction
            twist = self.move_forward(1, speed, twist)

        # turn left twist.angular.z is positive, turn right twist.angular.z is negative
        # turn left data.axes[0] is positive, turn right, data.axes[0] is negative
        angular_speed = data.axes[0]
        #angular_speed = angular_speed * -1 # for small joystick, inverted
        if angular_speed > 0:
            twist = self.move_sideway(angular_speed, self.lidar.flag_fl, self.lidar.flag_l, twist)
        if angular_speed < 0:
            twist = self.move_sideway(angular_speed, self.lidar.flag_fr, self.lidar.flag_r, twist)
        
        global clamp
        if clamp == False:
            self.pub.publish(twist)

    def move_forward(self, flag, multiplier, twist):
        if flag == 1:
            twist.linear.x = self.speed_fast * multiplier
        elif flag == 2:
            twist.linear.x = self.speed_slow * multiplier
        elif flag == 3:
            twist.linear.x = 0

        print("x: ", twist.linear.x)
        return twist

    def move_sideway(self, angular_speed, flag_frontside, flag_side, twist):
        if (flag_frontside == 3) & (flag_side >= 2):
            twist.angular.z = 0
        elif (flag_frontside <= 2) & (flag_side >= 2):
            twist.angular.z = self.speed_slow * angular_speed
        elif (flag_frontside == 1) & (flag_side == 1):
            twist.angular.z = self.speed_fast * angular_speed

        twist.angular.z = twist.angular.z * 2.5
        print("z: ", twist.angular.z)
        return twist

def get_gui(data = None):
    if data is None:
        settings = {
            "platform_stop_dist": 1.0,
            "platform_clear_dist": 1.6,
            "platform_normalSpeed": 0.2,
            "platform_slowDownSpeed": 0.1
        }
        return settings
    global lidar
    global joystick
    settings = json.loads(data)
    try:
        lidar.init_distance(settings)
        joystick.init_speed(settings)
    except:
        pass

def controllerCallback(data):
    global clamp
    print('triggered')
    if data.data//10 == 2:
        clamp = False
    elif data.data//10 == 3 or data.data//10 == 4:
        clamp = True

if __name__ == '__main__':
    rospy.init_node('controller_joystick')

    clamp = True
    settings = get_gui()
    global lidar
    global joystick
    lidar = LidarProcessor(settings)
    joystick = JoystickProcessor(settings,lidar)

    rospy.Subscriber('trigger_msgs', Int16, controllerCallback)
    rospy.Subscriber('gui_settings', String, get_gui)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        lidar.update_distance()
        lidar.update_speed()

        rate.sleep()
