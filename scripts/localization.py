#!/usr/bin/python3

# from std_msgs.msg import Header

# from nav_msgs.msg import Odometry
# from gazebo_msgs.msg import ModelStates


# def true_path(data):
#     global path
#     path.header = data.header
#     pose = PoseStamped()
#     pose.header = data.header
#     pose.pose = data.pose.pose
#     path.poses.append(pose)
#     path_pub.publish(path)




# path_pub = rospy.Publisher('/cube1/path', Path, queue_size=10)

# if __name__ == '__main__':
#     rospy.init_node('ekf')

#     path = Path()
#     # h = Header()
#     # h.stamp = rospy.Time.now()
#     # h.frame_id = "odom"
#     # h.seq = 0

#     # path.header = h

#     # odom_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, true_path)
#     odom_sub = rospy.Subscriber('/cube1/diff_drive_controller/odom', Odometry, true_path)

#     rospy.spin()

###############################################################################
# Postfiltering of pose estimate using extended Kalman filter.
# Due to highly accurate but low-frequent April tag pose estimation, predict
# the Kalman state open-loop with high frequency based on the last control inputs
# and "reset" it at every April tag update (meas_noise << proc_noise).
###############################################################################
import numpy as np
import rospy
import time
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import String, Int16MultiArray, Float32MultiArray
from sensor_msgs.msg import JointState

class Main():

    def __init__(self):
        cube = rospy.get_param('localization/robot_name')
        # self.start()
        self.wheel_radius = 0.01905
        # Initialize tf listener and pose/trajectory publisher.
        self.tf_listener = tf.TransformListener()

        self.pose_pub = rospy.Publisher(str("/" + cube + "/pose"), PoseWithCovarianceStamped, queue_size=1)

        self.traj_pub = rospy.Publisher(str("/" + cube + "/path"), Path, queue_size=1)
        self.traj = Path()

        self.input_sub = rospy.Subscriber(str("/" + cube + "/joint_states"), JointState, self.control_callback)

        self.tag_sub = rospy.Subscriber("/" + cube + "/apriltag", Float32MultiArray, self.tag_callback)

        rospy.spin()

    def control_callback(self, message):
        # rospy.loginfo(message.velocity)
        # rate = rospy.Rate(50)
        # rate.sleep()
        linear_velocity = np.array([message.velocity[0], message.velocity[1]])
        self.control_inputs = np.array(linear_velocity) * self.wheel_radius
        
        # rospy.loginfo(self.control_inputs)

    def tag_callback(self, message):
        self.reading = message.data



        

if __name__ == '__main__':
    rospy.init_node('localization', anonymous=True)
    
    Main()