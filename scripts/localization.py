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
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String, Int16MultiArray, Float32MultiArray, Bool, Float64
from sensor_msgs.msg import JointState
from kalman import RobotEKF
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import ModelStates, LinkStates

class Main():

    def __init__(self):
        self.cube = rospy.get_param('localization/robot_name')
        self.start()
        self.wheel_radius = 0.01905
        self.err_threshold = 0.3
        
        self.pose_pub = rospy.Publisher(str("/" + self.cube + "/pose"), PoseWithCovarianceStamped, queue_size=1)

        # Get control input
        rospy.Subscriber(str("/" + self.cube + "/joint_states"), JointState, self.control_callback, queue_size=1)
        
        # Get apriltag readings and run kalman update step
        rospy.Subscriber("/" + self.cube + "/apriltag", Float32MultiArray, self.tag_callback, queue_size=1)

        # Publish odom path
        self.odom_path_pub = rospy.Publisher(str("/" + self.cube + "/odom_path"), Path, queue_size=1)
        rospy.Subscriber(str("/"+ self.cube + "/diff_drive_controller/odom"), Odometry, self.odom_sub, queue_size=1)
        self.odom_path = Path()

        # Publish actual gazebo path
        self.true_path_pub = rospy.Publisher(str("/" + self.cube + "/path"), Path, queue_size=1)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.true_sub, queue_size=1)
        self.link_id = 0
        self.true_path = Path()
        self.true_path_arr = np.array([0, 0])       # Initialize to origin 
        
        # Publish kalman prediction path
        self.kalman_path_pub = rospy.Publisher(str("/" + self.cube + "/traj"), Path, queue_size=1)
        self.kalman_path = Path()
        self.kalman_path_arr = np.array([0, 0])     # Initialize to origin 

        # Get notified when finished square sequence
        rospy.Subscriber(str("/" + self.cube + "/square"), Bool, self.draw_err, queue_size=1)

        # Error publisher for plotting
        self.err_pub = rospy.Publisher(str("/" + self.cube + "/err"), Float64, queue_size=10)

        rospy.spin()
    
    def start(self):
        self.control_inputs = np.array([0, 0])

        # Initialize kalman filter to None
        self.kalman = None # RobotEKF(wheelbase=0.10082)
        self.last_update_time = rospy.get_time()

        # Update Kalman fitler timer - High frequent udpate
        self.olu_rate = 5   # Hz
        self.olu_timer = rospy.Timer(rospy.Duration(1.0/float(self.olu_rate)), self.open_loop_update)

    def shutdown(self):
        self.olu_timer.shutdown()

    def control_callback(self, data):
        """ Receive control inputs and convert it to linear velocity (m/s)
        :data: JointState message from /cube/joint_states topic
        """

        # Initialize EKF first time when first receive control input
        if self.kalman is None:
            self.kalman = RobotEKF(wheelbase=0.10082)

        angular_velocity = np.array([data.velocity[0], data.velocity[1]])

        # Convert angular velocity to linear velocity then set control_inputs accordingly
        self.control_inputs = angular_velocity * self.wheel_radius

    def open_loop_update(self, event):
        """ Intermediate high-frequent Kalman state prediction update based on control inputs
        :control inputs: np array ([left_wheel_velocity, right_wheel_velocity])
        """
        
        if self.kalman is None:
            return False
        
        prev_pose = self.kalman.pose
        dt = rospy.get_time() - self.last_update_time
        
        # Prediction step
        self.kalman.predict(self.control_inputs, dt)
        self.last_update_time = rospy.get_time()

        # Calculates error between EKF estimates and ground-truth
        err = self.calc_err(True, prev_pose)

        # Publishes kalman path
        self.pub_kalman()

        return True

    def tag_callback(self, data):
        """ Update step of EKF when AprilTag detected
        :data: Float32MultiArray from topic /cube/apriltag [ID, distance(m), bearing(rad)]
        """
        if self.kalman is None:
            return False
        
        prev_pose = self.kalman.pose
        tag_id, distance, yaw = data.data[0], data.data[1], data.data[2]

        landmark = {}
        landmark[0.0] = np.array([4.0, 0.0])
        landmark[1.0] = np.array([2.5, 4.0])
        landmark[2.0] = np.array([-1.5, 2.5])
        landmark[3.0] = np.array([0.0, -1.5])

        z = np.array([[distance], [yaw]])
        dt = rospy.get_time() - self.last_update_time

        u = np.array([0, 0])        #self.control_inputs
        
        # Reset prediction + update step
        self.kalman.predict(u, dt)
        self.kalman.update(z, landmark[tag_id], dt)
        self.last_update_time = rospy.get_time()

        # Calculates error between EKF estimates and ground-truth
        err = self.calc_err(False, prev_pose)

        # Publishes kalman path
        self.pub_kalman()

        return True

    def odom_sub(self, data):
        """ Publish odom path
        :data: Odometry message from /cube/diff_drive_controller/odom topic
        """
        self.odom_path.header = data.header

        pose = PoseStamped()
        pose.header = data.header
        pose.pose = data.pose.pose

        self.odom_path.poses.append(pose)
        self.odom_path_pub.publish(self.odom_path)

    def true_sub(self, data):
        """ Publish ground-truth gazebo path 
        :data: LinkStates message from /gazebo/link_states
        """
        # Find index number of cube link
        if self.link_id == 0:
            for i, link in enumerate(data.name):
                if link == str(self.cube + "::" + self.cube + "_dummy"):
                    self.link_id = i
        else:
            self.true_path.header.frame_id = "odom"
            self.true_path.header.seq += 1
            self.true_path.header.stamp = rospy.Time().now()

            pose = PoseStamped()
            pose.pose = data.pose[self.link_id]
            pose.header = self.true_path.header

            self.true_path.poses.append(pose)
            self.true_path_pub.publish(self.true_path)

    def pub_kalman(self):
        """ Publish EKF path
        """
        self.kalman_path.header.frame_id = "odom"
        self.kalman_path.header.seq += 1
        self.kalman_path.header.stamp = rospy.Time().now()

        pose = PoseStamped()
        pose.header = self.kalman_path.header
        quat = quaternion_from_euler(0.0, 0.0, self.kalman.pose[2])
        pose.pose.position.x = self.kalman.pose[0, 0]
        pose.pose.position.y = self.kalman.pose[1, 0]
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        self.kalman_path.poses.append(pose)
        self.kalman_path_pub.publish(self.kalman_path)
    
    def calc_err(self, stack, prev_pose): 
        """ Helper function to calculate error between ground-truth and EKF estimate
        :stack: bool variable to check whether to add to the path_arr or not for the final average
        """
        if not self.kalman_path.poses:
            return 0
        
        # Get the latest EKF pose estimation and ground-truth pose
        kalman_pose = np.array([self.kalman_path.poses[-1].pose.position.x,
                                self.kalman_path.poses[-1].pose.position.y])
        true_pose = np.array([self.true_path.poses[-1].pose.position.x,
                              self.true_path.poses[-1].pose.position.y])
        
        # Add pose estimations to arrays for avg err calculation
        if stack:
            self.kalman_path_arr = np.vstack((self.kalman_path_arr, kalman_pose))
            self.true_path_arr = np.vstack((self.true_path_arr, true_pose))
        
        # Calculate current error
        err = np.sqrt(np.sum((true_pose-kalman_pose)**2, axis=0))
        err = Float64(data=float(err))
        self.err_pub.publish(err)
        
        # If error too high, reset it accordingly
        if err.data > self.err_threshold:
            rospy.loginfo("WHOOPSIE")
            # Reset it to prev pose
            # self.kalman.pose = prev_pose 
            # self.kalman.subs[self.kalman.x_x] =  prev_pose[0,0]
            # self.kalman.subs[self.kalman.x_y] = prev_pose[1,0]
            # self.kalman.subs[self.kalman.x_theta] = prev_pose[2,0]

            # Reset it to latest odom reading, taking into account transformation with world frame
            x = self.odom_path.poses[-1].pose.position.x + 1.0
            y = self.odom_path.poses[-1].pose.position.y
            tx = self.odom_path.poses[-1].pose.orientation.x
            ty = self.odom_path.poses[-1].pose.orientation.y
            tz = self.odom_path.poses[-1].pose.orientation.z
            tw = self.odom_path.poses[-1].pose.orientation.w
            bearing = euler_from_quaternion([tx, ty, tz, tw])[2]
            self.kalman.pose = np.array([[x],[y],[bearing]])
            self.kalman.subs[self.kalman.x_x] = x # prev_pose[0,0]
            self.kalman.subs[self.kalman.x_y] = y #prev_pose[1,0]
            self.kalman.subs[self.kalman.x_theta] = bearing #prev_pose[2,0]

        return err
    
    def draw_err(self, data):
        """ Calculate average error when received the signal from square.py that sequence is done
        :data: Bool message from /cube/square topic
        """
        if data.data:
            curr_err = np.sqrt(np.sum((self.true_path_arr-self.kalman_path_arr)**2, axis=1))
            rospy.loginfo("AVG ERROR %s", np.mean(curr_err))

if __name__ == '__main__':
    rospy.init_node('localization', anonymous=True)
    
    Main()