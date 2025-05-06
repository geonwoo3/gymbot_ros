#!/usr/bin/env python3


import rospy
import serial
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from uwb_node import uwb_publisher, UWB, uwb_listener
import threading




def main():
    pub_uwb_thread = threading.Thread(target = uwb_publisher)
    pub_uwb_thread.start()

    rospy.sleep(1)


    uwb = UWB(subscriber=uwb_listener)


if __name__ == '__main__':
    main()
