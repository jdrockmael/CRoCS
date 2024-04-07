#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32MultiArray
from time import sleep

speed_pub = rospy.Publisher("robot_twist", Float32MultiArray, queue_size=1)
range_readings = (0.0, 0.0)
lock_on = True

def update_reading(cam : Float32MultiArray):
    global range_readings
    range_readings = (cam[2], cam[1])

def control_loop():
    heading_err = range_readings[2]
    distance_err = range_readings[1] - 0.05 # in meters

    if abs(distance_err) > 0.05 and abs(heading_err) > 0.01:
        
        linear = distance_err * 1
        angular_l = heading_err * 1
        angular_r = -heading_err * 1

        l_speed = linear + angular_l
        r_speed = linear + angular_r

        speed_pub.publish(Float32MultiArray(data=[l_speed, r_speed]))

        sleep(0.01)
    else:
        speed_pub.publish(Float32MultiArray(data=[0.0, 0.0]))

if __name__ == '__main__':
    rospy.init_node('lock')
    rospy.Subscriber("range", Float32MultiArray, update_reading)

    while not rospy.is_shutdown():
        if lock_on:
            control_loop()
    