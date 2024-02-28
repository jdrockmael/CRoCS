#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32MultiArray
from time import sleep
from drivers.motor_dr import Motor

motor_left = Motor(24, 23, 6, 5, 1)
motor_right = Motor(21, 20, 22, 27, -1)

def control_loop(data : Float32MultiArray):
    data = data.data
    delta_t = 0.001

    if len(data) != 0:
        heading_err = data[2] - 0.0
        distance_err = data[1] - 0.2 # in meters
        
        linear = distance_err * 5
        angular_l = heading_err * 1
        angular_r = -heading_err * 1

        l_eff = linear + angular_l
        r_eff = linear + angular_r

        motor_left.drive(l_eff)
        motor_right.drive(r_eff)

        sleep(delta_t)
    else:
        motor_left.stop()
        motor_right.stop()

if __name__ == '__main__':
    rospy.init_node('locking')

    rospy.Subscriber("range", Float32MultiArray, control_loop)

    rospy.spin()
