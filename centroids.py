import cv2
import sys
import numpy as np

cap = cv2.VideoCapture(0)

while(1):
    # Take each frame
    _, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_blue = np.array([104, 100, 225])
    upper_blue = np.array([130, 255, 255])

    # define range of red color in HSV
    lower_red = np.array([130, 165, 225])
    upper_red = np.array([179, 255, 255])

    # define range of green color in HSV
    lower_green = np.array([52, 110, 225])
    upper_green = np.array([84, 215, 255])

    # Threshold the HSV image to get only blue colors
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # Threshold the HSV image to get only red colors
    mask_red = cv2.inRange(hsv, lower_red, upper_red)

    # Threshold the HSV image to get only blue colors
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    # create master mask for centroids
    mask = cv2.bitwise_or(mask_blue,mask_red)
    mask = cv2.bitwise_or(mask, mask_green)

    # Bitwise-AND mask_f and original image
    res = cv2.bitwise_and(frame, frame, mask= mask)

    # convert the grayscale image to binary image
    ret,thresh = cv2.threshold(mask,127,255,0)
    # find contours in the binary image
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        # calculate moments for each contour
        M = cv2.moments(c)
        # calculate x,y coordinate of center
        # creates lower bound for centroids
        if M["m00"] >= 50:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0
        cv2.circle(res, (cX, cY), 5, (255, 255, 255), -1)
        cv2.putText(res, "centroid %d , %d with area %d" % (cX, cY, M["m00"]), (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # here's some code to detect which color is at that centroid
        if mask_blue[cY, cX] != 0:
            cv2.putText(res, "Blue", (cX - 25, cY),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        elif mask_red[cY, cX] != 0:
            cv2.putText(res, "Red", (cX - 25, cY),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        elif mask_green[cY, cX] != 0:
            cv2.putText(res, "Green", (cX - 25, cY),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
    

    cv2.imshow('frame',frame)
    cv2.imshow('mask_blue',mask_blue)
    cv2.imshow('mask_red',mask_red)
    cv2.imshow('mask_green',mask_green)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)
    #cv2.imshow('detects',im_with_keypoints)

    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()