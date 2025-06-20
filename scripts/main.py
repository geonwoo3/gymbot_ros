#!/usr/bin/env python3


import rospy
import serial
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from uwb_node import uwb_publisher, UWB
from lidar_node import Lidar
from visualizer import Visualizer
import threading



def main():
    rospy.init_node('main_controller', anonymous=True)

    rospy.sleep(1)

    uwb_thread = threading.Thread(target=uwb_publisher)
    uwb_thread.daemon = True
    uwb_thread.start()


    uwb = UWB(rate = 4)
    lidar = Lidar(delta_thresh = 0.2)
    visualizer = Visualizer(uwb, lidar)

    # Use rospy.Rate to manage loop frequency
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        visualizer.update_plot()
        rate.sleep()

    


if __name__ == '__main__':
    main()
