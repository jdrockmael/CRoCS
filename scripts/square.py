#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
import time
import math



def drive(linear, angular, sec):
    rate = rospy.Rate(100)
    time.sleep(0.5)
    t_end = time.time() + sec
    while time.time() < t_end:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        vel_pub.publish(msg)
        rate.sleep()

    msg = Twist()
    msg.linear.x = 0.0  
    msg.angular.z = 0.0
    vel_pub.publish(msg)
    
if __name__ == '__main__':
    rospy.init_node('square', anonymous=True)
    
    cube = rospy.get_param('square/robot_name')       
    vel_pub = rospy.Publisher(str("/" + cube + "/diff_drive_controller/cmd_vel"), Twist, queue_size=10)

    time.sleep(5)
    turn = math.pi / 4
    drive(0.5, 0, 5)
    drive(0, turn, 2)
    drive(0.5, 0, 5)
    drive(0, turn, 2)
    drive(0.5, 0, 5)
    drive(0, turn, 2)
    drive(0.5, 0, 5)
    drive(0, turn, 2)
    # t_end = time.time() + 5
    # while time.time() < t_end:
    #     msg = Twist()
    #     msg.linear.x = 0.5
    #     vel_pub.publish(msg)
    #     rate.sleep()

    # msg = Twist()
    # msg.linear.x = 0.0  
    # msg.angular.z = 0.0  
    # vel_pub.publish(msg)

    # time.sleep(0.5)
    # t_end = time.time() + 2
    # while time.time() < t_end:
    #     msg = Twist()
    #     msg.angular.z = math.pi / 4
    #     vel_pub.publish(msg)
    #     rate.sleep()

    # msg = Twist()
    # msg.linear.x = 0.0  
    # msg.angular.z = 0.0
    # vel_pub.publish(msg)

    # time.sleep(0.5)
    # t_end = time.time() + 5
    # while time.time() < t_end:
    #     msg = Twist()
    #     msg.linear.x = 0.5
    #     msg.angular.z = 0.0
    #     vel_pub.publish(msg)
    #     rate.sleep()

    