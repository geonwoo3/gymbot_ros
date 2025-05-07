#!/usr/bin/env python3

import matplotlib.pyplot as plt
import threading
import rospy


class Visualizer():
    def __init__(self, uwb, lidar):
        self.uwb = uwb
        self.lidar = lidar
        self.fig, self.ax = plt.subplots()
        plt.ion()

    def update_plot(self):
        self.ax.clear()
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)

        if self.uwb.get_target() is not None:
            x, y, _ = self.uwb.get_target()
            self.ax.plot(x, y, 'ro', label="Target")

        for x, y in self.lidar.get_obstacle_coordinates():
            self.ax.plot(x, y, 'b.', label="Obstacle")

        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.01)
# class Visualizer():
#     def __init__(self, uwb, lidar):
#         self.uwb = uwb
#         self.lidar = lidar
#         self.fig, self.ax = plt.subplots()
#         plt.ion()
#         threading.Thread(target=self.run, daemon=True).start()

#     def run(self):
#         while not rospy.is_shutdown():
#             self.ax.clear()
#             self.ax.set_xlim(-10, 10)
#             self.ax.set_ylim(-10, 10)

#             if self.uwb.get_target() is not None:
#                 x, y, _ = self.uwb.get_target()
#                 self.ax.plot(x,y,'ro', label="Target")
            
#             for x, y, in self.lidar.get_obstacle_coordinates():
#                 self.ax.plot(x,y,'b.', label="Obstacle")

#             self.ax.legend()
#             self.fig.canvas.draw()
#             self.fig.canvas.flush_events()
#             plt.pause(0.1)