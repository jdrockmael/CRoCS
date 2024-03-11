#!/usr/bin/python3
import rospy
from geometry_msgs.msg import PoseStamped, Twist, Vector3

pub = rospy.Publisher('/cube1/diff_drive_controller/cmd_vel', Twist, queue_size=1)

def drive_to(data : PoseStamped):
    rospy.logerr(data.pose.position)
    move_cmd = Twist()
    move_cmd.linear.x = 0.2
    for _ in range(1000):
        pub.publish(move_cmd)


if __name__ == '__main__':
    rospy.init_node('ik')

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, drive_to)

    rospy.spin()