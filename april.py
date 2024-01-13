import cv2
from dt_apriltags import Detector
import numpy as np
import glob
import math

visualization = True
try:
    import cv2
except:
    raise Exception('You need cv2 in order to run the demo. However, you can still use the library without it.')

try:
    from cv2 import imshow
except:
    print("The function imshow was not implemented in this installation. Rebuild OpenCV from source to use it")
    print("Visualization will be disabled.")
    visualization = False

try:
    import yaml
except:
    raise Exception('You need yaml in order to run the tests. However, you can still use the library without it.')


LINE_LENGTH = 5
CENTER_COLOR = (0, 255, 0)
CORNER_COLOR = (255, 0, 255)

############### CAMERA CALIBRATION #################
# Get camera parameters

# criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# objp = np.zeros((6*7,3), np.float32)
# objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
# # Arrays to store object points and image points from all the images.
# objpoints = [] # 3d point in real world space
# imgpoints = [] # 2d points in image plane.
# images = glob.glob('calib_data/*.jpg')c
# cam_param = []
# for fname in images:
   # img = cv2.imread(fname)
   # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
   # ret, corners = cv2.findChessboardCorners(gray, (7,6), None)
    # # If found, add object points, image points (after refining them)
   # if ret == True:
       # objpoints.append(objp)
       # corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
       # imgpoints.append(corners2)

       # ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
       # cam_param = ( mtx[0,0], mtx[1,1], mtx[0,2], mtx[1,2] )
# print(cam_param)


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

# plot a little text
def plotText(image, center, color, text):
    center = (int(center[0]) + 4, int(center[1]) - 4)
    return cv2.putText(image, str(text), center, cv2.FONT_HERSHEY_SIMPLEX,
                       1, color, 3)

cap = cv2.VideoCapture(0)
at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

at_detector = Detector()

# Arducam parameters
cam_param = (534.0708862263613, 534.1191479816413, 341.5340710724817, 232.94565221113476)
while(1):

    # Take each frame
    _, frame = cap.read()
    grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    img = grayscale
    tags =at_detector.detect(img, estimate_tag_pose=True, camera_params=cam_param, tag_size=5)
    # tags = at_detector.detect(img)

    # color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    
    if tags:
        # print(tags)
        for tag in tags:
            # print(tag)
            # Roll
            # print("YAWWWWWWWWWWWWW", math.degrees(math.atan2(tag.pose_R[1][0],tag.pose_R[0][0])))
            print("YAW", math.degrees(math.atan2(-tag.pose_R[2][0],math.sqrt(pow(tag.pose_R[2][1],2) + pow(tag.pose_R[2][2],2)))))
            # yaw=atan2(R(2,1),R(1,1));
            
            # print("tag_id: %s, center: %s" % (tag.tag_id, tag.center))
            frame = plotPoint(frame, tag.center, CENTER_COLOR)
            # frame = plotText(frame, tag.center, CENTER_COLOR, tag.tag_id)
            # frame = plotText(frame, tag.center, CENTER_COLOR, np.round(tag.pose_t[2], 3))
            # print(len(tag.corners))
            width = []
            for idx in range(len(tag.corners)):
                width.append(max(abs(tag.corners[idx-1, :] - tag.corners[idx,:])))
                # cv2.line(frame, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
            # print(width)
            #print(tag.pos_R)
            width = np.average(width)
            #print("FOCAL LENGTH: ", width * 60 / 5)
            #these values work at 1.5inx1.5in 360 and 60
            focal = 360
            manual_dist = focal * 60 / width
            
            # print(width)
            # Focal lenght = (Pixel weidth x dsitance) / Actual width
            #print("CALIB DIST", tag.pose_t[-1])
            print("MANUAL DIST", manual_dist, " mm")


    if visualization:
        cv2.imshow('Detected tags', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
