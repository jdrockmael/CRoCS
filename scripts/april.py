#!/usr/bin/python3
#/bin/python3.9
import cv2
from dt_apriltags import Detector
import numpy as np
import math
import collections
import rospy
from std_msgs.msg import Float32MultiArray, Bool
# from CRoCs.msg import April


import time

class AprilCam():
    def __init__(self):
        self.cam_param = (534.0708862263613, 534.1191479816413, 341.5340710724817, 232.94565221113476)
        self.cap = cv2.VideoCapture(0)
        self.at_detector = Detector()

        self.range_l = collections.deque(maxlen=10)
        self.bearing_l = collections.deque(maxlen=10)

        self.prevM = {}          # Dictionary of previous measurements for moving average
        self.start = time.time()
        meter = np.array([0.05, 0.08, 0.15, 0.2, 0.3])
        measure1x1 = np.array([0.304, 0.461, 0.886, 1.173, 1.761]) # Readings for 1x1 in tag
        self.m, self.b = np.polyfit(measure1x1, meter, 1)

    # Convert 1x1 april tag pose to real world measurements in m
    def pose2real(self, measurement):
        # Fit a linear model
        return self.m * measurement + self.b

    def get_measurements(self):
        # Take each frame
        _, frame = self.cap.read()
        grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


        tags_side =self.at_detector.detect(grayscale, estimate_tag_pose=True, camera_params=self.cam_param, tag_size=0.122)

        measurement = []
        if tags_side:
            for i, tag_side in enumerate(tags_side):
                id = tag_side.tag_id
                tag = tag_side
                x = tag.pose_t[0]
                z = tag.pose_t[2]

                # Convert reading to proper distance for 1x1 tags
                if id != 0:
                    x = self.pose2real(x)
                    z = self.pose2real(z)
                distance = z
                hor_distance = x
                range = math.sqrt(distance**2 + hor_distance**2)
                bearing = math.atan2(hor_distance, distance)

                if id not in self.prevM:
                    self.prevM[id] = {"range": collections.deque(maxlen=10), "bearing": collections.deque(maxlen=10)}

                self.prevM[id]["range"].append(range)
                self.prevM[id]["bearing"].append(bearing)
                
                # print(id)
                # Return an array of [ID, range(m), bearing(radian)]
                measurement.append(Float32MultiArray(data=[float(id), float(np.average(self.prevM[id]["range"])), float(np.average(self.prevM[id]["bearing"]))]))

        # self.end = time.time()
        # print("Elapased time: ", self.end - self.start)
        # self.start = time.time()
        return measurement

    def stop(self):
        cv2.destroyAllWindows()

def measure():
    range_pub = rospy.Publisher('range', Float32MultiArray, queue_size=10)
    no_tag_pub = rospy.Publisher('no_tag', Bool, queue_size=10)
    rospy.init_node("april")

    rospy.loginfo("Starting Arducam node")
    cam = AprilCam()

    while not rospy.is_shutdown():
        rospy.sleep(0.005)              # Sleep for 5ms
        tags = cam.get_measurements()                                                                                                                                                              
        if tags:
            for measurement in tags:
                range_pub.publish(measurement)
        else:
            no_tag_pub.publish(True)

if __name__ == "__main__":
    try:
        measure()
    except rospy.ROSInterruptException:
        pass
    
