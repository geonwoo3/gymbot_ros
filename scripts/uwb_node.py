#!/usr/bin/env python3

import rospy
import serial
import numpy as np
from std_msgs.msg import String
from scipy.optimize import minimize, NonlinearConstraint

anchor_port = {}
pub = None

def read_from_anchor(event):
    for anchor_name, port in anchor_port.items():
        try:
            line = port.readline().decode('utf-8').strip()
            if line:
                msg = f"{anchor_name} {line}"

                rospy.loginfo(msg)
                pub.publish(msg)
        except Exception as e:
            rospy.logerr(f"Error reading from {anchor_name}: {e}")

def uwb_publisher():
    global anchor_port, pub

    # Publisher for UWB
    pub = rospy.Publisher('/uwb_distance', String, queue_size=10)

    try:
        anchor_port = {
            "A1": serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1),
            "A2": serial.Serial('/dev/ttyUSB2', 115200, timeout=0.1),
            "A3": serial.Serial('/dev/ttyUSB3', 115200, timeout=0.1)
        }
    except Exception as e:
        rospy.logerr(f"Failed to connect to one of the serial ports: {e}")
        return

    # Set up a timer to read from UWB anchor every 0.33 seconds (~3Hz)
    rospy.Timer(rospy.Duration(1.0 / 4), read_from_anchor)
    rospy.spin()




def estimate_target(anchors, anchor_centers):
    try:
        distances = [anchors["A1"], anchors["A2"], anchors["A3"]]
        c1 = anchor_centers[0]
        c2 = anchor_centers[1] 
        c3 = anchor_centers[2]

        def objective(vars):
            x = vars[0:3]
            s1 = vars[3:6]
            s2 = vars[6:9]
            s3 = vars[9:12]
            return np.linalg.norm(x - s1) + np.linalg.norm(x - s2) + np.linalg.norm(x - s3)
        
        def constraint_sphere1(vars): return np.linalg.norm(vars[3:6] - c1) - distances[0]
        def constraint_sphere2(vars): return np.linalg.norm(vars[6:9] - c2) - distances[1]
        def constraint_sphere3(vars): return np.linalg.norm(vars[9:12] - c3) - distances[2]

        constraints = [
            NonlinearConstraint(constraint_sphere1, 0, 0),
            NonlinearConstraint(constraint_sphere2, 0, 0),
            NonlinearConstraint(constraint_sphere3, 0, 0)
        ]

        x0 = np.concatenate([
            np.mean(anchor_centers, axis = 0),
            c1 + np.array([distances[0], 0, 0]),
            c2 + np.array([distances[1], 0, 0]),
            c3 + np.array([distances[2], 0, 0]),
        ])

        result = minimize(objective, x0, constraints = constraints, method="SLSQP")

        if not result.success:
            rospy.loginfo("Optimization failed")
            return Non
        target = result.x[0:3]

        return target
    except Exception as e:
        rospy.logerr(f"error was : {e}")
        return None

class UWB:

    def __init__(self,
                rate=4
                ):
        self.anchors = {}
        self.target_coordinate = None
        self.traj = []
        self.rate = rospy.Rate(rate)
        rospy.Subscriber('/uwb_distance', String, self.uwb_callback)
        # Anchor positions
        self.anchor_centers = [
            np.array([0,  np.sqrt(3)/2, 0]),
            np.array([-1, -np.sqrt(3)/2, 0]),
            np.array([1,  -np.sqrt(3)/2, 0])   
        ]


    def uwb_callback(self, msg):
        try:
            parts = msg.data.split()
            rospy.loginfo(f"Message parts: {parts}")
            if len(parts) >= 2:

                anchor_id, distance = parts[0], parts[1]
                rospy.loginfo(anchor_id, distance)
                self.anchors[anchor_id] = float(distance)
                rospy.loginfo(f"Full dict: {self.anchors}")

                self.target_coordinate = estimate_target(self.anchors, self.anchor_centers)
                rospy.loginfo(f"Target: ", {self.target_coordinate})
        except Exception as e:
            rospy.logwarn(f"[UWB Class] Failed to parse msg: {msg.data}, error: {e}")

    def update_anchor(self):
        return
    
    def get_anchors(self):
        return self.anchors
    
    def get_trajectory(self):
        return self.traj

    def get_target(self):
        return self.target_coordinate

