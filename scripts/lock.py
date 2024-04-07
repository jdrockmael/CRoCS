#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32MultiArray
from time import sleep

speed_pub = rospy.Publisher("robot_twist", Float32MultiArray, queue_size=1)

def control_loop(cam : Float32MultiArray):
    data = cam.data
    delta_t = 0.05
    distance_to_tag = 0.2

    if len(data) != 0:
        heading_err = data[2]
        distance_err = data[1] - distance_to_tag # in meters
        
        linear = distance_err * 5
        angular_l = heading_err * 1
        angular_r = -heading_err * 1

        l_speed = linear + angular_l
        r_speed = linear + angular_r

        speed_pub.publish(Float32MultiArray(data=[l_speed, r_speed]))

        sleep(delta_t)
    else:
        speed_pub.publish(Float32MultiArray(data=[0.0, 0.0]))

if __name__ == '__main__':
    rospy.init_node('lock')
    rospy.Subscriber("range", Float32MultiArray, control_loop)
    rospy.spin()