#!/usr/bin/python3
import rospy
from math import sin, cos
from std_msgs.msg import Float32MultiArray
from time import clock_gettime
from drivers.motor_dr import motor

pose = [0.0, 0.0, 0.0]

pose_pub = rospy.Publisher('curr_pose', Float32MultiArray, queue_size= 1)

def fk():
    global pose
    curr = None
    l = 0.101 # meters
    prev = ((0.0, 0.0), clock_gettime(0))

    curr = (motor.get_distance(), clock_gettime(0))

    vl = (curr[0][0] - prev[0][0]) / (curr[1] - prev[1])
    vr = (curr[0][1] - prev[0][1]) / (curr[1] - prev[1])

    prev = curr

    r = (l / 2) * ((vl + vr) / (vr - vl))
    omega = (vr - vl) / l

    x, y, theta = pose[0], pose[1], pose[2]
    icc_x = x - r * sin(theta)
    icc_y = y + r * cos(theta)

    new_x = (x - icc_x) * cos()

if __name__ == '__main__':
    rospy.init_node('fk')

    while rospy.is_shutdown():
        pass