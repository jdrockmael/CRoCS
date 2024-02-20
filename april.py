import cv2
from dt_apriltags import Detector
import numpy as np
import math
import collections
import rospy
from std_msgs.msg import Float32MultiArray
import time

class AprilCam():
    def __init__(self):
        self.cam_param = (534.0708862263613, 534.1191479816413, 341.5340710724817, 232.94565221113476)
        self.cap = cv2.VideoCapture(0)
        self.at_detector = Detector()

        self.range_l = collections.deque(maxlen=10)
        self.bearing_l = collections.deque(maxlen=10)
       
        
        #self.start = time.time()
    def get_measurements(self):
        # Take each frame
        _, frame = self.cap.read()
        grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


        tags_side =self.at_detector.detect(grayscale, estimate_tag_pose=True, camera_params=self.cam_param, tag_size=0.02)

        #tags_face = self.at_detector.detect(grayscale, estimate_tag_pose=True, camera_params=self.cam_param, tag_size=0.122)

        measurement = []
        if tags_side:
            for i, tag_side in enumerate(tags_side):
                tag = tag_side
                if tag_side.tag_id == 0:
                    #tag = tags_face[i]
                    tag = self.at_detector.detect(grayscale, estimate_tag_pose=True, camera_params=self.cam_param, tag_size=0.122)[i]
                distance = tag.pose_t[2]
                hor_distance = tag.pose_t[0] 
                range = math.sqrt(distance**2 + hor_distance**2)
                bearing = math.degrees(math.atan2(hor_distance, distance))

                # Moving average distance and yaw reading
                self.range_l.append(range)
                self.bearing_l.append(bearing)

                avg_range = round(np.average(self.range_l), 2)
                avg_bearing = round(np.average(self.bearing_l), 2)

                measurement.append(Float32MultiArray(data=[float(tag.tag_id), float(avg_range), float(avg_bearing), float(2)]))
                # return {
                #     "ID": tag.tag_id,
                #     "Range": avg_range, # mm
                #     "Bearing": avg_bearing, # Degrees
                #     "AprilID": 2
                # }
        #self.end = time.time()
        #print("Elapased time: ", self.end - self.start)
        #self.start = time.time()
        return measurement

    def stop(self):
        cv2.destroyAllWindows()

def measure():
    range_pub = rospy.Publisher('range', Float32MultiArray, queue_size=10)
    rospy.init_node("april")

    rospy.loginfo("Starting Arducam node")
    cam = AprilCam()

    while not rospy.is_shutdown():
        tags = cam.get_measurements()                                                                                                                                                              
        if tags:
            for measurement in tags:
                range_pub.publish(measurement)

if __name__ == "__main__":
    try:
        measure()
    except rospy.ROSInterruptException:
        pass
    
