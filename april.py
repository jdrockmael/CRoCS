import cv2
from dt_apriltags import Detector
import numpy as np
import math
import collections
import rospy
from std_msgs.msg import Float32MultiArray

class AprilCam():
    def __init__(self):
        self.cam_param = (534.0708862263613, 534.1191479816413, 341.5340710724817, 232.94565221113476)
        self.cap = cv2.VideoCapture(0)
        self.at_detector = Detector()

        mm = np.array([200, 250, 300, 350, 400])

        pixel6x6 = np.array([7.88, 9.45, 11.52, 14.1, 15.6]) # Cam pixel readings of 6x6
        self.m, self.b = {}, {}
        self.m[6], self.b[6] = np.polyfit(pixel6x6, mm, 1)

        self.range_l = collections.deque(maxlen=10)
        self.bearing_l = collections.deque(maxlen=10)

    # Convert april tag pose to real world measurements in mm
    def pose2real(self, measurement, length):
        # Fit a linear model
        return self.m[length] * measurement + self.b[length]

    def get_measurements(self):
        # Take each frame
        _, frame = self.cap.read()
        grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        tags =self.at_detector.detect(grayscale, estimate_tag_pose=True, camera_params=self.cam_param, tag_size=5)
        
        if tags:
            for tag in tags:                    
                distance = self.pose2real(tag.pose_t[2], 6)      
                hor_distance = self.pose2real(tag.pose_t[0], 6) # 25.4mm is the real world distance from center of april tag to edge
                range = math.sqrt(distance**2 + hor_distance**2)
                bearing = math.degrees(math.atan2(hor_distance, distance))

                # Moving average distance and yaw reading
                self.range_l.append(range)
                self.bearing_l.append(bearing)

                avg_range = round(np.average(self.range_l), 2)
                avg_bearing = round(np.average(self.bearing_l), 2)

                print(tag.tag_id)
                print(avg_range)
                return Float32MultiArray(data=[float(tag.tag_id), float(avg_range), float(avg_bearing), float(2)])
                # return {
                #     "ID": tag.tag_id,
                #     "Range": avg_range, # mm
                #     "Bearing": avg_bearing, # Degrees
                #     "AprilID": 2
                # }
        else:
            return None

    def stop(self):
        cv2.destroyAllWindows()

def measure():
    range_pub = rospy.Publisher('range', Float32MultiArray, queue_size=10)
    rospy.init_node("april")

    rospy.loginfo("Starting Arducam node")
    cam = AprilCam()

    while not rospy.is_shutdown():
        measurement = cam.get_measurements()                                                                                                                                                              
        if measurement is not None:
            range_pub.publish(measurement)

if __name__ == "__main__":
    try:
        measure()
    except rospy.ROSInterruptException:
        pass
    