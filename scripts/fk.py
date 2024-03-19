#!/usr/bin/python3
import rospy
from math import sin, cos
from std_msgs.msg import Float32MultiArray

pose = [0.0, 0.0, 0.0]
pose_pub = rospy.Publisher('curr_pose', Float32MultiArray, queue_size= 1)

def calc_fk(wheel_vel : Float32MultiArray, delta_t):
    global pose
    l = 0.101 # meters
    vl, vr = wheel_vel.data

    omega = (vr - vl) / l
    vel = (vr + vl) / 2

    x, y, theta = pose[0], pose[1], pose[2]

    new_x = x + delta_t * vel * cos(theta)
    new_y = y + delta_t * vel * sin(theta)
    new_theta = theta + delta_t * omega

    pose = [new_x, new_y, new_theta]
    pose_pub.publish(Float32MultiArray(data=pose))

if __name__ == '__main__':
    delta_t = 0.01 # seconds
    rospy.init_node('fk')
    rospy.Subscriber('wheel_vel', Float32MultiArray, calc_fk, callback_args=delta_t)
    rospy.spin()
    