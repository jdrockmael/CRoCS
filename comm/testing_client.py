import rospy
from std_msgs.msg import String

pub = rospy.Publisher('out_going_msgs', String, queue_size=10)

if __name__ == '__main__':
    # init node
    rospy.init_node('take_input')
    
    # pubs the incoming messages from server
    while not rospy.is_shutdown():
        data = input()
        pub.publish(data)