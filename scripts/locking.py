#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32MultiArray
from time import sleep
from drivers.motor_dr import Motor

motor = Motor()

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

        motor.drive(l_eff, r_eff)

        sleep(delta_t)
    else:
        motor.stop()

if __name__ == '__main__':
    rospy.init_node('locking')

    rospy.Subscriber("range", Float32MultiArray, control_loop)

    rospy.spin()
