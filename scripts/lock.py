#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32MultiArray
from time import sleep

speed_pub = rospy.Publisher("robot_twist", Float32MultiArray, queue_size=1)
cam_readings = (0.0, 0.0) # heading and distance
lock_on = True

def update_reading(cam : Float32MultiArray):
    global cam_readings
    cam_readings = (cam.data[2], cam.data[1])

def control_loop():
    lin_p = 1
    lin_i = 0.5
    lin_d = 0

    ang_p = 1
    ang_i = 0.5

    tolerance = 0.02
    delta_t = 0.05

    linear_area = 0.0
    angular_area = 0.0
    
    while(abs(cam_readings[1]) > tolerance and abs(cam_readings[0]) > tolerance and lock_on):
        curr_distance = cam_readings[1]
        curr_err_heading = cam_readings[0]
        
        linear_area += curr_distance * delta_t
        
        linear_eff_p = curr_distance * lin_p
        linear_eff_i = linear_area * lin_i
        linear_eff_d = ((curr_distance - prev_distance)/delta_t) * lin_d

        linear_eff = linear_eff_p + linear_eff_i + linear_eff_d

        angular_area += curr_err_heading * delta_t

        angular_eff_p = curr_err_heading * ang_p
        angular_eff_i = angular_area * ang_i

        angular_eff = angular_eff_p + angular_eff_i

        speed_pub.publish(Float32MultiArray(data=[linear_eff, angular_eff]))

        prev_distance = curr_distance
        sleep(delta_t)

    speed_pub.publish(Float32MultiArray(data=[0.0, 0.0]))

if __name__ == '__main__':
    rospy.init_node('lock')
    rospy.Subscriber("range", Float32MultiArray, update_reading)

    while not rospy.is_shutdown():
        control_loop()
    