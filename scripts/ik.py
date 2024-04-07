#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32MultiArray
from time import sleep
from math import atan2, sqrt, pi

curr_pose = [0.0, 0.0, 0.0]
speed_pub = rospy.Publisher("robot_twist", Float32MultiArray, queue_size=1)

def calc_angle_diff(desired, actual):
    diff = desired - actual
    if diff >= pi:
        diff = diff - 2*pi
    elif diff <= -pi:
        diff = 2*pi + diff
    return diff

def calc_distance(from_point, to_point):
    curr_distance = sqrt(pow(to_point[0]-from_point[0], 2) + pow(to_point[1]-from_point[1], 2))
    vector_heading = atan2(to_point[1] - from_point[1], to_point[0] - from_point[0])
    diff_in_heading = calc_angle_diff(vector_heading, curr_pose[2])

    if diff_in_heading > pi/2 or diff_in_heading < -pi/2:
        return -curr_distance
    else:
        return curr_distance

def update_pose(pose : Float32MultiArray):
    global curr_pose
    curr_pose = pose.data

def drive_to(pose):
    prev_distance = calc_distance((curr_pose[0], curr_pose[1]), (pose[0], pose[1]))
    desired_heading = atan2(pose[1] - curr_pose[1], pose[0] - curr_pose[0])
    desired_heading = 2*pi + desired_heading if desired_heading < 0 else desired_heading

    lin_p = 1
    lin_i = 0.5
    lin_d = 0

    ang_p = 1
    ang_i = 0.5

    tolerance = 0.02
    delta_t = 0.05

    linear_area = 0.0
    angular_area = 0.0

    done = 1
    
    while(abs(prev_distance) > tolerance):
        curr_distance = calc_distance((curr_pose[0], curr_pose[1]), (pose[0], pose[1]))
        curr_err_heading = calc_angle_diff(desired_heading, curr_pose[2])
        
        linear_area += curr_distance * delta_t * done
        
        linear_eff_p = curr_distance * lin_p
        linear_eff_i = linear_area * lin_i
        linear_eff_d = ((curr_distance - prev_distance)/delta_t) * lin_d

        linear_eff = linear_eff_p + linear_eff_i + linear_eff_d

        done = 0 if linear_eff > 0.2  or linear_eff < -0.2 else 1

        angular_area += curr_err_heading * delta_t

        angular_eff_p = curr_err_heading * ang_p
        angular_eff_i = angular_area * ang_i

        angular_eff = angular_eff_p + angular_eff_i

        speed_pub.publish(Float32MultiArray(data=[linear_eff, angular_eff]))

        prev_distance = curr_distance
        sleep(delta_t)

    speed_pub.publish(Float32MultiArray(data=[0.0, 0.0]))

def turn_to(heading): 
    prev_error = calc_angle_diff(heading, curr_pose[2])
    tolerance = 0.1
    delta_t = 0.05

    p = 1
    i = 0.5
    d = 0

    area = 0.0

    while(abs(prev_error) > tolerance):
        curr_error = calc_angle_diff(heading, curr_pose[2])
        area += curr_error * delta_t
        
        angular_err_p = curr_error *p
        angular_err_i = area * i
        angular_err_d = ((curr_error-prev_error)/delta_t) * d

        omega = angular_err_p + angular_err_i + angular_err_d

        speed_pub.publish(Float32MultiArray(data=[0.0, omega]))

        prev_error = curr_error
        sleep(delta_t)
    
    speed_pub.publish(Float32MultiArray(data=[0.0, 0.0]))

def drive_to_point(desired : Float32MultiArray):
    desired_pose = desired.data
    desired_heading = atan2(desired_pose[1] - curr_pose[1], desired_pose[0] - curr_pose[0])
    desired_heading = 2*pi + desired_heading if desired_heading < 0 else desired_heading

    turn_to(desired_heading)
    drive_to(desired_pose)
    turn_to(desired_pose[2])

if __name__ == '__main__':
    rospy.init_node('ik')
    #rospy.Subscriber("range", Float32MultiArray, control_loop)
    rospy.Subscriber("curr_pose", Float32MultiArray, update_pose)
    rospy.Subscriber("target_pose", Float32MultiArray, drive_to_point)
    rospy.spin()