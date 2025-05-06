#!/usr/bin/env python3


import rospy
import serial
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from uwb_node import uwb_publisher, UWB, uwb_listener
from lidar_node import Lidar
import threading



def main():
    rospy.init_node('main_controller', anonymous=True)

    rospy.sleep(1)


    uwb = UWB(rate = 4)
    lidar = Lidar(delta_thresh = 0.2)

    rospy.spin()


if __name__ == '__main__':
    main()
