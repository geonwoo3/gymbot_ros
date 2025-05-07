#!/usr/bin/env python3


import rospy
import serial
import numpy as np
import math
from std_msgs.msg import String
from scipy.optimize import minimize, NonlinearConstraint
from sensor_msgs.msg import LaserScan


def adaptive_sample(ranges, angle_min, angle_increment, range_min, range_max, delta_threash=0.2):
    sampled = []
    angle = angle_min
    prev_valid = None

    for r in ranges:
        if math.isnan(r) or r < range_min or r > range_max:
            angle += angle_increment
            continue
        if prev_valid is None or abs(r - prev_valid) > delta_threash:
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            sampled.append([x, y])
            prev_valid = r        
        
        angle += angle_increment

    return sampled

class Lidar():
    def  __init__(self,
                  delta_thresh = 0.2):
        self.delta_thresh = delta_thresh
        self.obstacle_coordinates = []
        rospy.Subscriber('/scan', LaserScan, self.callback)

    def callback(self, data):
        self.obstacle_coordinates = adaptive_sample(data.ranges,
                                                    data.angle_min,
                                                    data.angle_increment,
                                                    data.range_min,
                                                    data.range_max,
                                                    self.delta_thresh)
        # rospy.loginfo(self.obstacle_coordinates)

    def get_obstacle_coordinates(self):
        return self.obstacle_coordinates