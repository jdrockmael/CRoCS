#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32MultiArray
from time import sleep
from math import atan2, sqrt

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
    distance = sqrt(pow(pose[0]-curr_pose[0], 2) + pow(pose[1]-curr_pose[1], 2))
    heading = atan2(pose[1], pose[0])
    
    while(distance > 0.1):
        distance = sqrt(pow(pose[0]-curr_pose[0], 2) + pow(pose[1]-curr_pose[1], 2))
        
        linear = distance * 5.0
        angular_l = curr_pose[2] - heading * 1.0
        angular_r = -curr_pose[2] + heading * 1.0

        l_eff = linear + angular_l
        r_eff = linear + angular_r

        eff_pub.publish(Float32MultiArray(data=[0, 0]))

        sleep(0.001)

    eff_pub.publish(Float32MultiArray(data=[0, 0]))

def turn_to(heading):
    desired_heading = heading
    error = curr_pose[2] - desired_heading

    while(error > 0.1):
        angular_l = error * 1.0
        angular_r = -error * 1.0

        l_eff = angular_l
        r_eff = angular_r

        eff_pub.publish(Float32MultiArray(data=[0, 0]))

        sleep(0.001)
    
    eff_pub.publish(Float32MultiArray(data=[0, 0]))

def drive_to_point(desired : Float32MultiArray):
    desired_pose = desired.data
    desired_heading = atan2(desired_pose[1], desired_pose[0])
    turn_to(desired_heading)
    drive_to(desired_pose)
    turn_to(desired_pose[2])

if __name__ == '__main__':
    rospy.init_node('ik')
    #rospy.Subscriber("range", Float32MultiArray, control_loop)
    rospy.Subscriber("curr_pose", Float32MultiArray, update_pose)
    rospy.Subscriber("target_pose", Float32MultiArray, drive_to_point)
    rospy.spin()