#!/usr/bin/python3
import rospy
from time import sleep
from std_msgs.msg import Float32MultiArray
from threading import Lock

desired_vel = (0.0, 0.0)
curr_vel = [0.0, 0.0]

desired_lock = Lock()
vel_lock = Lock()
    
eff_pub = rospy.Publisher("wheel_eff", Float32MultiArray, queue_size=1)

def update_desired(desired : Float32MultiArray):
    global desired_vel
    linear, angular = desired.data
    
    l = 0.101 # meters
    desired_vl = linear - ((angular * l)/2)
    desired_vr = linear + ((angular * l)/2)

    with desired_lock:
        desired_vel = (desired_vl, desired_vr)

    what = "setting global to", desired_vel
    rospy.logerr(what)

def update_vel(wheel_vel : Float32MultiArray):
    global curr_vel
    with vel_lock:
        curr_vel = wheel_vel.data

def speed_controller():
    tolerance = 0.05
    delta_t = 0.01

    p = 0.3
    i = 0.5
    d = 0
    
    desired_l, desired_r = (0, 0)
    curr_l, curr_r = (0, 0)

    with vel_lock:
        curr_l, curr_r = curr_vel
    with desired_lock:
        desired_l, desired_r = desired_vel

    curr_eff = (0.0, 0.0)
    prev_error = (desired_l - curr_l, desired_r - curr_r)
    area = (0.0, 0.0)
    prev_desired = (desired_l, desired_r)

    while not rospy.is_shutdown():
        with vel_lock:
            curr_l, curr_r = curr_vel
        with desired_lock:
            desired_l, desired_r = desired_vel

        if prev_desired != (desired_l, desired_r):
            prev_error = (desired_l -  curr_l, desired_r - curr_r)
            area = (0.0, 0.0)
            prev_desired = (desired_l, desired_r)

            huh = "now going to", prev_desired
            rospy.logerr(huh)

        if abs(prev_error[0]) > tolerance or abs(prev_error[1]) > tolerance:
            curr_error = (desired_l - curr_l, desired_r - curr_r)
            area_l = area[0] + ( 0.5 * (curr_error[0] + prev_error[0]) * delta_t)
            area_r = area[1] + ( 0.5 * (curr_error[1] + prev_error[1]) * delta_t)
            area = (area_l, area_r)

            l_proportion = curr_error[0] * p
            l_integral = area[0] * i
            l_derivative = ((curr_error[0] - prev_error[0]) / delta_t) * d

            r_proportion = curr_error[1] * p
            r_integral = area[1] * i
            r_derivative = ((curr_error[1] - prev_error[1]) / delta_t) * d

            left_eff = curr_eff[0] + l_proportion + l_integral + l_derivative
            right_eff = curr_eff[1] + r_proportion + r_integral + r_derivative

            curr_eff = (left_eff, right_eff)
            prev_error = curr_error

            eff_pub.publish(Float32MultiArray(data=[left_eff, right_eff]))

        sleep(delta_t)

if __name__ == '__main__':
    rospy.init_node('locomotion')
    rospy.Subscriber("robot_twist", Float32MultiArray, update_desired)
    rospy.Subscriber("wheel_vel", Float32MultiArray, update_vel)
    speed_controller()
