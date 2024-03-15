#!/usr/bin/python3
import rospy
from math import sin, cos
from std_msgs.msg import Float32MultiArray
from time import clock_gettime
from drivers.motor_dr import motor

pose_pub = rospy.Publisher('curr_pose', Float32MultiArray, queue_size= 1)

def calc_wheel_vel(prev_wheel_dis, curr_wheel_dis, delta_t):
    delta_l = curr_wheel_dis[0] - prev_wheel_dis[0]
    delta_r = curr_wheel_dis[1] - prev_wheel_dis[1]

    left_vel = delta_l / delta_t
    right_vel = delta_r / delta_t

    return (left_vel, right_vel)

def calc_fk(wheel_vel, curr_pose, delta_t):
    l = 0.101 # meters
    vl, vr = wheel_vel

    omega = (vr - vl) / l
    vel = (vr + vl) / 2
    # r = (l / 2.0) * ((vl + vr) / (vr - vl))

    x, y, theta = curr_pose[0], curr_pose[1], curr_pose[2]
    # icc_x = x - r * sin(theta)
    # icc_y = y + r * cos(theta)

    # new_x = (x - icc_x) * cos(omega * delta_t) + (y - icc_y) * -sin(omega * delta_t) + icc_x
    # new_y = (x - icc_x) * sin(omega * delta_t) + (y - icc_y) * cos(omega * delta_t) + icc_y
    # new_theta = theta + omega * delta_t

    new_x = x + delta_t * vel * cos(theta)
    new_y = y + delta_t * vel * sin(theta)
    new_theta = theta + delta_t * omega

    return [new_x, new_y, new_theta]

if __name__ == '__main__':
    rospy.init_node('fk')

    pose = [0.0, 0.0, 0.0]
    prev_dist = (0.0, 0.0)
    prev_t = clock_gettime(0)

    while not rospy.is_shutdown():
        curr_dist = motor.get_distance()
        curr_t = clock_gettime(0)

        dt = curr_t - prev_t

        wheel_vel = calc_wheel_vel(prev_dist, curr_dist, dt)
        pose = calc_fk(wheel_vel, pose, dt)

        prev_dist = curr_dist
        prev_t = curr_t

        pose_pub.publish(Float32MultiArray(data=pose))