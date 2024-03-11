#!/usr/bin/python3
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
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String, Int16MultiArray, Float32MultiArray, Header
from sensor_msgs.msg import JointState
from kalman import RobotEKF
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import ModelStates, LinkStates

class Main():

    def __init__(self):
        cube = rospy.get_param('localization/robot_name')
        self.start()
        self.wheel_radius = 0.01905
        self.tf_listener = tf.TransformListener()

        self.pose_pub = rospy.Publisher(str("/" + cube + "/pose"), PoseWithCovarianceStamped, queue_size=1)

        # Get control input and run kalman prediction step
        rospy.Subscriber(str("/" + cube + "/joint_states"), JointState, self.control_callback)
        
        # Run kalman update step
        rospy.Subscriber("/" + cube + "/apriltag", Float32MultiArray, self.tag_callback)

        # self.true_path_pub = rospy.Publisher(str("/" + cube + "/path"), Path, queue_size=10)
        # self.odom_sub = rospy.Subscriber(str("/"+ cube + "/diff_drive_controller/odom"), Odometry, self.true_path)
        self.true_path_pub = rospy.Publisher(str("/" + cube + "/path"), Path, queue_size=1)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.true_path, queue_size=1)
        # True path
        self.path = Path()

        self.traj_pub = rospy.Publisher(str("/" + cube + "/traj"), Path, queue_size=1)
        # Kalman path
        self.traj = Path()

        rospy.spin()
    
    def start(self):
        # cube = rospy.get_param('localization/robot_name')
        self.olu_rate = 4   # Hz
        self.control_inputs = np.array([0, 0])
        self.kalman = RobotEKF()
        self.last_update_time = rospy.get_time()

        self.olu_timer = rospy.Timer(rospy.Duration(1.0/float(self.olu_rate)), self.open_loop_update)

    def shutdown(self):
        self.olu_timer.shutdown()

    def true_path(self, data):
        self.path.header.frame_id = "odom"
        self.path.header.seq += 1
        self.path.header.stamp = rospy.Time().now()

        pose = PoseStamped()
        pose.pose = data.pose[5]
        pose.header = self.path.header
        self.path.poses.append(pose)
        self.true_path_pub.publish(self.path)

    def traj_path(self):
        self.traj.header.frame_id = "odom"
        self.traj.header.seq += 1
        self.traj.header.stamp = rospy.Time().now()
        pose = PoseStamped()
        pose.header = self.traj.header

        quat = quaternion_from_euler(0.0, 0.0, self.kalman.pose[2])
        pose.pose.position.x = self.kalman.pose[0][0] #/ 1000.0
        pose.pose.position.y = self.kalman.pose[1][0] #/ 1000.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        self.traj.poses.append(pose)
        self.traj_pub.publish(self.traj)

    def open_loop_update(self, event):
        ''' Intermediate high-frequent Kalman state update based on
        prediction internal vehicle model (predict) and control inputs. '''
        
        u = self.control_inputs
        dt = rospy.get_time() - self.last_update_time
        self.kalman.predict(u, dt)
        self.last_update_time = rospy.get_time()
        self.traj_path()
        return True
    
    def control_callback(self, message):
        # rospy.loginfo(message.velocity)
        # rate = rospy.Rate(50)
        # rate.sleep()
        linear_velocity = np.array([message.velocity[0], message.velocity[1]])
        self.control_inputs = np.array(linear_velocity) * self.wheel_radius

    def tag_callback(self, message):
        reading = message.data
        tag_id, distance, yaw = reading[0], reading[1], reading[2]
        # landmark = np.array([3.0, 0.0])
        landmark = {}
        landmark[0.0] = np.array([4.0, 0.0])
        landmark[1.0] = np.array([2.5, 4.0])
        landmark[2.0] = np.array([-1.5, 2.5])
        landmark[3.0] = np.array([0.0, -1.5])
        z = np.array([[distance], [yaw]])
        dt = rospy.get_time() - self.last_update_time
        self.kalman.update(z, landmark[tag_id], dt)
        self.last_update_time = rospy.get_time()
        self.traj_path()
        return True
        

if __name__ == '__main__':
    rospy.init_node('localization', anonymous=True)
    
    Main()