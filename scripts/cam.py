#!/usr/bin/python3
#/bin/python3.9
import cv2
from dt_apriltags import Detector
import math
import rospy
from std_msgs.msg import Float32MultiArray
from time import sleep
# from CRoCs.msg import April


import time

class AprilCam():
    def __init__(self):
        self.cam_param = (534.0708862263613, 534.1191479816413, 341.5340710724817, 232.94565221113476)
        self.cap = cv2.VideoCapture(0)
        self.at_detector = Detector()

        self.start = time.time()

    def get_measurements(self):
        # Take each frame
        _, frame = self.cap.read()
        grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


        tags_side =self.at_detector.detect(grayscale, estimate_tag_pose=True, camera_params=self.cam_param, tag_size=0.122)

        measurement = []
        if tags_side:
            for i, tag_side in enumerate(tags_side):
                id = tag_side.tag_id
                tag = None
                if (id % 4 != 0):
                    tag = self.at_detector.detect(grayscale, estimate_tag_pose=True, camera_params=self.cam_param, tag_size=0.02)[i] #V0.01632857142
                else:
                    tag = tag_side

                x = tag.pose_t[0]
                z = tag.pose_t[2]

                # Convert reading to proper distance for 1x1 tags
                # if id != 0:
                #     x = self.pose2real(x)
                #     z = self.pose2real(z)
                distance = z
                hor_distance = x
                hyp = math.sqrt(distance**2 + hor_distance**2)  # Range to april tag
                bearing = math.atan2(hor_distance, distance)

                # print(id)
                # Return an array of [ID, range(m), bearing(radian)]
                measurement.append(Float32MultiArray(data=[float(id), float(hyp), float(bearing)]))

        # self.end = time.time()
        # print("Elapased time: ", self.end - self.start)
        # self.start = time.time()
        return measurement

    def stop(self):
        cv2.destroyAllWindows()

def measure():
    range_pub = rospy.Publisher('range', Float32MultiArray, queue_size=1)
    rospy.init_node("cam")

    rospy.loginfo("Starting Arducam node")
    cam = AprilCam()

    while not rospy.is_shutdown():
        sleep(0.05)              # Sleep for 5ms
        tags = cam.get_measurements()
        if tags:
            for measurement in tags:
                range_pub.publish(measurement)
                rospy.logerr(measurement)

if __name__ == "__main__":
    try:
        measure()
    except rospy.ROSInterruptException:
        pass
    
