import os
import cv2
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.animation import FuncAnimation
import sys
from copy import copy
from math import cos, sin, atan, asin, pi
import math

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import os

class OccupancyMap(object):
    occupancyMap = np.zeros((2,2))
    def __init__(self):
        self.resolution = 1
        self.mapWidth   = 2
        self.mapHeight  = 2
        self.origin     = [0, 0]
        rospy.Subscriber("map", OccupancyGrid, callback=self.callback)

    def callback(self, message):
        self.resolution   = message.info.resolution
        self.mapWidth     = message.info.width
        self.mapHeight    = message.info.height
        self.occupancyMap = np.reshape(np.asarray(message.data), (self.mapWidth, self.mapHeight))  
        self.origin       = [message.info.origin.position.x, message.info.origin.position.y]

    def getIntepolationCoordinate(self, x, y):
        # output: x0, x1, y0, y1
        x_temp = x - self.origin[0]
        y_temp = y - self.origin[1]
        x_c = x_temp/self.resolution
        y_c = y_temp/self.resolution
        return int(math.floor(x_c))*self.resolution, int(math.ceil(x_c))*self.resolution, int(math.floor(y_c))*self.resolution, int(math.ceil(y_c))*self.resolution

    def getOccP(self, x, y):
        return self.occupancyMap[int(round(x/self.resolution)), int(round(y/self.resolution))]

class LaserSubs(object):
    laser_coordinate = None
    lp = lg.LaserProjection()

    def __init__(self):
        # rospy.Subscriber('/base_scan', LaserScan, self.LaserData)
        rospy.Subscriber('scan', LaserScan, self.callback)

    def callback(self,msg):
        pc2_msg = self.lp.projectLaser(msg)
        point_generator = pc2.read_points_list(pc2_msg)
        self.laser_coordinate = np.asarray(point_generator)[:, :2]
    
    def get_laser_data(self):
        return self.laser_coordinate

class PoseArrayClass(object):
    # (x, y, z, x, y, z, w) position and quaternion
    poses = np.zeros((7,2))
    def __init__(self, radius = 0.0):
        rospy.Subscriber("particlecloud", PoseArray, callback=self.callback)
        # rospy.Subscriber("Pose_Array?", PoseArray, callback=self.calllback)
        self.radius = radius
    
    def callback(self, message):
        poses = message.poses
        poses = [[poses[i].position.x, poses[i].position.y, poses[i].position.z, poses[i].orientation.x, poses[i].orientation.y, poses[i].orientation.z, poses[i].orientation.w] for i in range(len(poses))]
        self.poses = np.asarray(poses)
        # print("PoseArray callback")

    def is_conversed(self):
        # unsure how to define convergence, currently using std of position
        # print(np.max(np.std(self.poses[:, :3], axis=1)))
        return np.max(np.std(self.poses[:, :3], axis=1)) < self.radius

class PostProcessPose:
    def __init__(self, pose_array_radius, threshold_radius, num_iterations):
        self.occupancyMap = OccupancyMap()
        self.laserSubs    = LaserSubs()
        self.poseArray    = PoseArrayClass(pose_array_radius)
        self.num_iterations = num_iterations
        self.threshold_radius = threshold_radius
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callback=self.callback)
        self.pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped)
    
    
    def derivative_of_location(self, angle, x, y):
        # base on equation (8) in the paper
        # output is (2, 3) matrix
        result = np.zeros((2,3))
        result[:, 0:2] = np.identity(2)
        result[0, 2]   = -math.sin(angle)*x - math.cos(angle)*y
        result[1, 2]   =  math.cos(angle)*x - math.sin(angle)*y
        return result
    
    def derivative_of_map_occupancy(self, x, y):
        # base on equation 10
        # output is a (2, 1) matrix
        x0, x1, y0, y1 = self.occupancyMap.getIntepolationCoordinate(x, y)
        occupancyMapTmp = self.occupancyMap
        result = np.zeros((1, 2))
        if (x1 == x0 or y1 == y0):
            raise Exception("Cannot compute derivative_of_map_occupancy")
        P11 = occupancyMapTmp.getOccP(x1,y1)
        P01 = occupancyMapTmp.getOccP(x0,y1)
        P10 = occupancyMapTmp.getOccP(x1,y0)
        P00 = occupancyMapTmp.getOccP(x0,y0)

        result[0, 0] = (y-y0)/(y1-y0)*(P11 - P01) + (y1-y)/(y1-y0)*(P10 - P00)
        result[0, 1] = (x-x0)/(x1-x0)*(P11 - P10) + (x1-x)/(x1-x0)*(P01 - P00)
        return result
    
    def compute_map_occupancy(self, x, y):
        x0, x1, y0, y1 = self.occupancyMap.getIntepolationCoordinate(x, y)
        if (x1 == x0 or y1 == y0):
            raise Exception("Cannot compute delta: divide by zero exception")
        occupancyMapTmp = self.occupancyMap
        P11 = occupancyMapTmp.getOccP(x1,y1)
        P01 = occupancyMapTmp.getOccP(x0,y1)
        P10 = occupancyMapTmp.getOccP(x1,y0)
        P00 = occupancyMapTmp.getOccP(x0,y0)
        occupancyProb  = (y-y0)/(y1-y0)*((x-x0)/(x1-x0)*P11 + (x1-x)/(x1-x0)*P01)
        occupancyProb += (y1-y)/(y1-y0)*((x-x0)/(x1-x0)*P10 + (x1-x)/(x1-x0)*P00)
        return occupancyProb

    def compute_Hessian_Matrix(self, data):
        #assuming I have a list of data points
        #the data is in shape of (n, 5)
        #a row is (angle, x_lidar, y_lidar, x, y)
        matrix = np.asarray([np.dot(self.derivative_of_map_occupancy(data[i, 3], data[i, 4]) , self.derivative_of_location(data[i, 0], data[i, 1], data[i, 2])) for i in range(len(data))])
        #matrix should have shape (n, 1, 3)
        result = np.zeros((3, 3))
        for i in range(len(matrix)):
            result += np.dot(np.transpose(matrix[i]) , matrix[i])
        return result
    
    def computeDelta1(self, data):
        res = np.dot(self.derivative_of_map_occupancy(data[3], data[4]), self.derivative_of_location(data[0], data[1], data[2]))
        res = (1-self.compute_map_occupancy(data[3], data[4])) * res
        return res

    def computeDelta(self, data):
        # data should be in shape (n, 5)
        # a row is (angle, x_lidar, y_lidar, x, y)
        # result is (x, y, angle) of the robot 
        matrix = np.asarray([self.computeDelta1(data[i]) for i in range(len(data))])        
        matrix = np.sum(matrix, axis=0)
        #matrix now should have shape (1,3)
        hessianMatrix = self.compute_Hessian_Matrix(data)
        # print(hessianMatrix)
        result = np.dot(np.linalg.inv(hessianMatrix) , np.transpose(matrix) )
        return result.flatten()

    def get_data_list(self, x_amcl, y_amcl, angle_amcl):
        laser_data = self.laserSubs.get_laser_data()
        num_points = laser_data.shape[0]
        A = np.asarray([[math.cos(angle_amcl), -math.sin(angle_amcl)], [math.sin(angle_amcl), math.cos(angle_amcl)]])
        B = np.asarray([x_amcl, y_amcl])
        S = np.dot(laser_data, np.transpose(A)) + np.repeat(np.reshape(B, (1,2)), num_points, axis=0)
        angle_stack = np.repeat(np.asarray([[angle_amcl]]), num_points, axis=0)
        return np.hstack((angle_stack, laser_data, S))

    def callback(self, message):
        (roll, pitch, angle) = euler_from_quaternion([message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z, message.pose.pose.orientation.w])
        # print([roll, pitch, angle])
        x = message.pose.pose.position.x
        y = message.pose.pose.position.y
        x_orig = x
        y_orig = y
        angle_orig = angle
        if (self.poseArray.is_conversed()):
            iteration = 0
            while (iteration < self.num_iterations):
                data  = self.get_data_list(x, y, angle)
                print(data)
                delta = self.computeDelta(data)
                x += delta[0]
                y += delta[1]
                angle += delta[2]
                iteration += 1
            if (math.sqrt((x-x_orig)*(x-x_orig) + (y-y_orig)*(y-y_orig)) < self.threshold_radius):
                p = PoseWithCovarianceStamped()
                p.header.stamp = rospy.get_rostime()
                p.header.frame_id = "map"
                p.pose.pose.position.x = x
                p.pose.pose.position.y = y
                p.pose.pose.position.z = 0
                quaternion = quaternion_from_euler(0, 0, angle)
                p.pose.pose.quaternion.x = quaternion[0]
                p.pose.pose.quaternion.y = quaternion[1]
                p.pose.pose.quaternion.z = quaternion[2]
                p.pose.pose.quaternion.w = quaternion[3]
                print("Publish new pose")
                self.pub.publish(p)


if __name__ == '__main__':
    # p = PostProcessPose()
    # print(p.derivative_of_map_occupancy(3.14/2, 0.005, 0.005))
    rospy.init_node('Post_Process', anonymous=True)
    # occcupancyMap = OccupancyMap()
    # poseArray = PoseArrayClass()
    # laserSubs = LaserSubs()
    PostProcessPose(1, 3, 10)
    rospy.spin()


    
    
