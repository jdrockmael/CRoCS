import rospy
from std_msgs.msg import Float32MultiArray
from time import sleep

pub = rospy.Publisher("robot_twist", Float32MultiArray, queue_size=1)
#pub = rospy.Publisher("target_pose", Float32MultiArray, queue_size=1)

if __name__ == '__main__':
    rospy.init_node('motor_test')
    #pub.publish(Float32MultiArray(data=[0.0, 0.0, 3.14]))
    pub.publish(Float32MultiArray(data=[0.0, 3.14]))
    sleep(1)
    pub.publish(Float32MultiArray(data=[0, 0]))