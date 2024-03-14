#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
import time
import math
from std_msgs.msg import Bool

def drive(distance, speed):
    # Drive foward in m/s
    rate = rospy.Rate(100)
    time.sleep(0.5)
    sec = distance / speed
    t_end = time.time() + sec
    while time.time() < t_end:
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0
        vel_pub.publish(msg)
        rate.sleep()

    msg = Twist()
    msg.linear.x = 0.0  
    msg.angular.z = 0.0
    vel_pub.publish(msg)

def turn(speed):
    # Speed in rad/s

    # Get how many seconds to turn by pi/2 / rad/s
    sec = math.pi / 2 / speed
    rate = rospy.Rate(100)
    time.sleep(1.0)
    t_end = time.time() + sec
    while time.time() < t_end:
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = speed
        vel_pub.publish(msg)
        rate.sleep()

    msg = Twist()
    msg.linear.x = 0.0  
    msg.angular.z = 0.0
    vel_pub.publish(msg)
    
def square():
    turn_speed = math.pi / 16
    forward = 2.5
    forward_speed = 0.1
    drive(1.5, forward_speed)
    turn(turn_speed)
    drive(forward, forward_speed)
    turn(turn_speed)
    drive(forward, forward_speed)
    turn(turn_speed)
    drive(forward, forward_speed)
    turn(turn_speed)
    
if __name__ == '__main__':
    rospy.init_node('square', anonymous=True)
    
    cube = rospy.get_param('square/robot_name')       
    vel_pub = rospy.Publisher(str("/" + cube + "/diff_drive_controller/cmd_vel"), Twist, queue_size=10)
    square_pub = rospy.Publisher(str("/" + cube + "/square"), Bool, queue_size=1)

    time.sleep(5)
    square()
    done = Bool(data=True)
    if done.data:
        rospy.loginfo(done)
    square_pub.publish(done)
