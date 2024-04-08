#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32MultiArray
from time import sleep

speed_pub = rospy.Publisher("robot_twist", Float32MultiArray, queue_size=1)
cam_readings = None # heading and distance
lock_on = True

def update_reading(cam : Float32MultiArray):
    global cam_readings
    cam_readings = (cam.data[2], cam.data[1])

def control_loop():
    prev_error = cam_readings[0]
    tolerance = 0.1
    delta_t = 0.05

    p = 1
    i = 0.5
    d = 0

    area = 0.0
    rospy.logerr(cam_readings)
    while((abs(cam_readings[0]) > tolerance or abs(cam_readings[1]) > 0.5) and lock_on):
        curr_error = cam_readings[0]
        area += curr_error * delta_t
        
        angular_err_p = curr_error *p
        angular_err_i = area * i
        angular_err_d = ((curr_error-prev_error)/delta_t) * d

        omega = angular_err_p + angular_err_i + angular_err_d

        speed_pub.publish(Float32MultiArray(data=[0.0, omega]))

        prev_error = curr_error
        sleep(delta_t)
    
    speed_pub.publish(Float32MultiArray(data=[0.0, 0.0]))

if __name__ == '__main__':
    rospy.init_node('lock')
    rospy.Subscriber("range", Float32MultiArray, update_reading)
    while cam_readings == None:
        pass
    control_loop()
    rospy.spin()
    