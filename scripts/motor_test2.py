import rospy
from std_msgs.msg import Float32MultiArray
from time import sleep

pub = rospy.Publisher("wheel_effort", Float32MultiArray, queue_size=1)
#pub = rospy.Publisher("target_pose", Float32MultiArray, queue_size=1)

if __name__ == '__main__':
    rospy.init_node('motor_test')
    #pub.publish(Float32MultiArray(data=[0.1, 0.2, 3.14]))
    pub.publish(Float32MultiArray(data=[1, 1]))
    sleep(1)
    pub.publish(Float32MultiArray(data=[0, 0]))