#!/usr/bin/env python3

import matplotlib.pyplot as plt
import threading

class Visualizer():
    def __init__(self, uwb, lidar):
        self.uwb = uwb
        self.lidar = lidar
        self.fig, self.ax = plt.subplots()
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while not rospy.is_shutdown():
            self.ax.clear()
            self.ax.set_xlim(-10, 10)
            self.ax.set_ylim(-10, 10)

            if self.uwb.get_target() is not None:
                x, y, _ = self.uwb.get_target()
                self.ax.plot(x,y,'ro', label="Target")
            
            for x, y, in self.lidar.get_obstacle_coordinates():
                self.ax.plot(x,y,'b.')

            self.ax.legend()
            plt.pause(0.1)