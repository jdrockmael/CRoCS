#!/usr/bin/python3
import rospy
from time import sleep
from std_msgs.msg import Float32MultiArray

desired_vel = (None, None)
curr_vel = [0.0, 0.0]
    
eff_pub = rospy.Publisher("wheel_eff", Float32MultiArray, queue_size=1)

def update_desired(desired : Float32MultiArray):
    global desired_vel
    linear, angular = desired.data
    
    l = 0.101 # meters
    desired_vl = linear - ((angular * l)/2)
    desired_vr = linear + ((angular * l)/2)

    desired_vel = (desired_vl, desired_vr)

def update_vel(wheel_vel : Float32MultiArray):
    global curr_vel
    curr_vel = wheel_vel.data

def speed_controller():
    global desired_vel
    tolerance = 0.05
    delta_t = 0.05

    p = 0.3
    i = 0.5

    curr_eff = (0.0, 0.0)
    area = (0.0, 0.0)
    prev_desired = (None, None)

    while not rospy.is_shutdown():
        if desired_vel != (None, None):
            curr_l, curr_r = curr_vel
            desired_l, desired_r = desired_vel
            curr_error = (desired_l - curr_l, desired_r - curr_r)

            if prev_desired != (desired_l, desired_r):
                area = (0.0, 0.0)
                prev_desired = (desired_l, desired_r)

            if abs(curr_error[0]) > tolerance or abs(curr_error[1]) > tolerance:
                area = (area[0] + curr_error[0] * delta_t, area[1] + curr_error[1] * delta_t)

                left_eff = curr_eff[0] + curr_error[0] * p + area[0] * i
                right_eff = curr_eff[1] + curr_error[1] * p + area[1] * i

                curr_eff = (left_eff, right_eff)

                eff_pub.publish(Float32MultiArray(data=[left_eff, right_eff]))
            else:
                desired_vel = (None, None)

            sleep(delta_t)

if __name__ == '__main__':
    rospy.init_node('locomotion')
    rospy.Subscriber("robot_twist", Float32MultiArray, update_desired)
    rospy.Subscriber("wheel_vel", Float32MultiArray, update_vel)
    speed_controller()
