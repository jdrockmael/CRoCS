#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32MultiArray
from time import sleep
from math import atan2, sqrt, sin, cos
import numpy as np

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

def update_pose(pose : Float32MultiArray):
    global curr_pose
    curr_pose = pose.data

def drive_to(pose):
    prev_distance = sqrt(pow(pose[0]-curr_pose[0], 2) + pow(pose[1]-curr_pose[1], 2))
    heading = atan2(pose[1], pose[0])

    lin_p = 0.3 
    lin_i = 0
    lin_d = 0.5

    ang_p = 0.3 
    ang_i = 0
    ang_d = 0.5

    tolerance = 0.1
    delta_t = 0.01

    linear_area = 0.0
    angular_area = 0.0
    
    while(prev_distance > tolerance or prev_distance < -tolerance):
        curr_distance = sqrt(pow(pose[0]-curr_pose[0], 2) + pow(pose[1]-curr_pose[1], 2))
        
        linear_area += 0.5 * (curr_distance + prev_distance) * delta_t
        
        linear_eff_p = curr_distance * lin_p
        linear_eff_i = linear_area * lin_i
        linear_eff_d = ((curr_distance - prev_distance)/delta_t) * lin_d

        linear_eff = linear_eff_p + linear_eff_i + linear_eff_d

        angular_area += 0.5 * (curr_pose[2] + heading) * delta_t

        angular_eff_p = curr_pose[2] - heading * ang_p
        angular_eff_i = angular_area * ang_i
        angular_eff_d = ((curr_pose[2] - heading)/delta_t) * ang_d

        angular_eff = angular_eff_p + angular_eff_i + angular_eff_d

        effort = [linear_eff, angular_eff]

        eff_pub.publish(Float32MultiArray(data=effort))

        prev_distance = curr_distance
        sleep(delta_t)

    eff_pub.publish(Float32MultiArray(data=effort))

def turn_to(heading):
    prev_error = heading - curr_pose[2]
    tolerance = 0.1
    delta_t = 0.01

    p = 0.3
    i = 0
    d = 0.5

    area = 0.0

    while(prev_error > tolerance or prev_error < -tolerance):
        curr_error = heading - curr_pose[2]

        area += 0.5 * (curr_error + prev_error) * delta_t
        
        angular_err_p = curr_error *p
        angular_err_i = area * i
        angular_err_d = ((curr_error-prev_error)/delta_t) * d

        effort = angular_err_p + + angular_err_i + angular_err_d

        eff_pub.publish(Float32MultiArray(data=[0.0, effort]))

        prev_error = curr_error
        sleep(delta_t)
    
    eff_pub.publish(Float32MultiArray(data=[0.0,effort]))

def calc_transform(pose):
    x = curr_pose[0]
    y = curr_pose[1]
    theta = curr_pose[2]
    transform = np.array([[cos(theta), -sin(theta), x],
                        [sin(theta), cos(theta), y],
                        [0.0, 0.0, 1.0]])

    new_pose = np.array([[pose[0]], [pose[1]], [1.0]])

    return np.dot(transform, new_pose)

def drive_to_point(desired : Float32MultiArray):
    desired_pose = desired.data
    transformed_pose = calc_transform(desired.data)
    desired_heading = atan2(transformed_pose[0], transformed_pose[1])

    turn_to(desired_heading)
    drive_to(desired_pose)
    turn_to(desired_pose[2])

if __name__ == '__main__':
    rospy.init_node('ik')
    #rospy.Subscriber("range", Float32MultiArray, control_loop)
    rospy.Subscriber("curr_pose", Float32MultiArray, update_pose)
    rospy.Subscriber("target_pose", Float32MultiArray, drive_to_point)
    rospy.spin()