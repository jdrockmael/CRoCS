import cv2
from dt_apriltags import Detector
import numpy as np
import glob
import math
import collections
import platform
from gpiozero import PhaseEnableMotor, Servo
#motor forward ccw, backward cw
from time import sleep

servo = Servo(25)
motor1 = PhaseEnableMotor(26, 13)
motor2 = PhaseEnableMotor(24, 23)

servo.min()
sleep(1)
servo.max()
motor1.forward(1)
motor2.backward(1)
sleep(3)


ardu = False
if platform.node()[0:4] == "croc":
    ardu = True

cam_param = None
if not ardu:
    from cv2 import imshow

    LINE_LENGTH = 5
    CENTER_COLOR = (0, 255, 0)
    CORNER_COLOR = (255, 0, 255)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    images = glob.glob('calib_data/*.jpg')
    
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (7,6), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
            cam_param = ( mtx[0,0], mtx[1,1], mtx[0,2], mtx[1,2] )

    ### Some utility functions to simplify drawing on the camera feed
    # draw a crosshair
    def plotPoint(image, center, color):
        center = (int(center[0]), int(center[1]))
        image = cv2.line(image,
                        (center[0] - LINE_LENGTH, center[1]),
                        (center[0] + LINE_LENGTH, center[1]),
                        color,
                        3)
        image = cv2.line(image,
                        (center[0], center[1] - LINE_LENGTH),
                        (center[0], center[1] + LINE_LENGTH),
                        color,
                        3)
        return image
else:
    # Arducam parameters
    cam_param = (534.0708862263613, 534.1191479816413, 341.5340710724817, 232.94565221113476)

# Convert april tag pose to real world measurements in mm
def pose2real(measurement):
    mm = np.array([20, 25, 30, 35, 40])
    # inches = np.array([10, 15, 18, 20])
    measurements = np.array([120, 142, 170, 186, 213]) # 6x6
    # measurements = np.array([7.88, 9.45, 11.52, 14.1, 15.6]) Alternate value# 6x6

    # Fit a linear model
    coefficients = np.polyfit(measurements, mm, 1)
    m, b = coefficients

    return m * measurement + b

yaw_l = collections.deque(maxlen=10)
dist_l = collections.deque(maxlen=10)
hor_dist_l = collections.deque(maxlen=10)
range_l = collections.deque(maxlen=10)
bearing_l = collections.deque(maxlen=10)

cap = cv2.VideoCapture(0)
at_detector = Detector()

while(1):

    # Take each frame
    _, frame = cap.read()
    grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    tags =at_detector.detect(grayscale, estimate_tag_pose=True, camera_params=cam_param, tag_size=5)

    # color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    
    if tags:
        # print(tags)
        for tag in tags:
            # print("tag_id: %s, center: %s" % (tag.tag_id, tag.center))
            if not ardu: 
                frame = plotPoint(frame, tag.center, CENTER_COLOR)
            # width = []
            # for idx in range(len(tag.corners)):
            #     width.append(max(abs(tag.corners[idx-1, :] - tag.corners[idx,:])))
            # width = np.average(width)

            # Focal length = (Pixel width x distance) / Actual width
            # Dist = (Actual Width x Focal Length) / Pixel Width
            #print("FOCAL LENGTH: ", width * 60 / 5)
            #these values work at 1.5inx1.5in 360 and 60
            # focal = 360
            # manual_dist = focal * 60 / width
            # curr_roll = math.degrees(math.atan2(tag.pose_R[1][0],tag.pose_R[0][0]))

            center_hor = 0 # real world distance of half the length april tag side in mm
            if tag.tag_id == 0:
                center_hor = 55.5
            elif tag.tag_id == 1:
                center_hor = 10
            elif tag.tag_id == 2:
                center_hor = 12
            elif tag.tag_id == 3:
                center_hor = 15
            else:
                center_hor = 60.5
                
            yaw = math.degrees(math.atan2(-tag.pose_R[2][0],math.sqrt(pow(tag.pose_R[2][1],2) + pow(tag.pose_R[2][2],2))))
            distance = pose2real(tag.pose_t[2])      
            hor_distance = pose2real(tag.pose_t[0]) - center_hor # 25.4mm is the real world distance from center of april tag to edge
            hyp = math.sqrt(distance**2 + hor_distance**2)
            bearing = math.degrees(math.atan2(hor_distance, distance))

            # Moving average distance and yaw reading
            yaw_l.append(yaw)
            dist_l.append(distance)
            hor_dist_l.append(hor_distance)
            range_l.append(hyp)
            bearing_l.append(bearing)

            avg_yaw = round(np.average(yaw_l), 2)
            avg_dist = round(np.average(dist_l), 2)
            avg_hor_dist = round(np.average(hor_dist_l), 2)
            avg_range = round(np.average(range_l), 2)
            avg_bearing = round(np.average(bearing_l), 2)

            print("ID ",tag.tag_id)
            print("YAW ", avg_yaw, " degree")
            print("Distance ", avg_dist, " mm")
            print("Horizontal distance", avg_hor_dist, " mm")
            print("Range ", avg_range, " mm")
            print("Bearing ", avg_bearing, " degree")


    # Disable visualization for running on server
    if not ardu:
        cv2.imshow('Detected tags', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
