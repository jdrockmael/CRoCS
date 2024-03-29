#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32MultiArray
from time import sleep
from math import atan2, sqrt, sin, cos, pi

curr_pose = [0.0, 0.0, 0.0]
eff_pub = rospy.Publisher("robot_twist", Float32MultiArray, queue_size=1)

# def control_loop(cam : Float32MultiArray):
#     data = cam.data
#     delta_t = 0.001

#     if len(data) != 0:
#         heading_err = data[2] - 0.0
#         distance_err = data[1] - 0.2 # in meters
        
#         linear = distance_err * 5
#         angular_l = heading_err * 1
#         angular_r = -heading_err * 1

#         l_eff = linear + angular_l
#         r_eff = linear + angular_r

#         motor.drive(l_eff, r_eff)

#         sleep(delta_t)
#     else:
#         motor.stop()

def calc_transform(pose):
    x = curr_pose[0]
    y = curr_pose[1]
    theta = curr_pose[2]

    new_x = cos(theta) * pose[0] + -sin(theta) * pose[1] + x
    new_y = sin(theta) * pose[0] + cos(theta) * pose[1] + y

    return (new_x, new_y)

def calc_angle_diff(desired, actual):
    diff = desired - actual
    if diff >= 180:
        diff = 360 - diff
    elif diff <= -180:
        diff = 360 + diff
    return diff

def update_pose(pose : Float32MultiArray):
    global curr_pose
    curr_pose = pose.data

def drive_to(pose):
    prev_distance = sqrt(pow(pose[0]-curr_pose[0], 2) + pow(pose[1]-curr_pose[1], 2))
    transformed_pose = calc_transform(pose)
    heading = curr_pose[2] + atan2(transformed_pose[1], transformed_pose[0])

    lin_p = 1
    lin_i = 0
    lin_d = 0

    ang_p = 1.5

    tolerance = 0.1
    delta_t = 0.05

    linear_area = 0.0
    
    while(prev_distance > tolerance or prev_distance < -tolerance):
        curr_distance = sqrt(pow(pose[0]-curr_pose[0], 2) + pow(pose[1]-curr_pose[1], 2))
        curr_err_heading = calc_angle_diff(heading, curr_pose[2])
        
        linear_area += curr_distance * delta_t
        
        linear_eff_p = curr_distance * lin_p
        linear_eff_i = linear_area * lin_i
        linear_eff_d = ((curr_distance - prev_distance)/delta_t) * lin_d

        linear_eff = linear_eff_p + linear_eff_i + linear_eff_d

        angular_eff = curr_err_heading * ang_p

        eff_pub.publish(Float32MultiArray(data=[linear_eff, angular_eff]))

        prev_distance = curr_distance
        sleep(delta_t)

    eff_pub.publish(Float32MultiArray(data=[0.0, 0.0]))

def turn_to(heading): 
    prev_error = calc_angle_diff(heading, curr_pose[2])
    tolerance = 0.02
    delta_t = 0.05

    p = 3
    i = 0.1
    d = 0

    area = 0.0

    while(abs(prev_error) > tolerance):
        curr_error = calc_angle_diff(heading, curr_pose[2])

        area += curr_error * delta_t
        
        angular_err_p = curr_error *p
        angular_err_i = area * i
        angular_err_d = ((curr_error-prev_error)/delta_t) * d

        omega = angular_err_p + angular_err_i + angular_err_d

        eff_pub.publish(Float32MultiArray(data=[0.0, omega]))

        prev_error = curr_error
        sleep(delta_t)
    
    eff_pub.publish(Float32MultiArray(data=[0.0, 0.0]))

def drive_to_point(desired : Float32MultiArray):
    desired_pose = desired.data
    transformed_pose = calc_transform(desired_pose)
    desired_heading = curr_pose[2] + atan2(transformed_pose[1], transformed_pose[0])

    turn_to(desired_heading)
    drive_to(desired_pose)
    turn_to(desired_pose[2])

if __name__ == '__main__':
    rospy.init_node('ik')
    #rospy.Subscriber("range", Float32MultiArray, control_loop)
    rospy.Subscriber("curr_pose", Float32MultiArray, update_pose)
    rospy.Subscriber("target_pose", Float32MultiArray, drive_to_point)
    rospy.spin()