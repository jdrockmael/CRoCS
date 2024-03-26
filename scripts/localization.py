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
        self.init_x = rospy.get_param('localization/x_pos')
        self.init_y = rospy.get_param('localization/y_pos')

        self.start()
        
        self.pose_pub = rospy.Publisher(str("/" + self.cube + "/pose"), PoseWithCovarianceStamped, queue_size=1)

        # Get control input
        rospy.Subscriber(str("/" + self.cube + "/joint_states"), JointState, self.control_callback, queue_size=1)

        # Get other cube pose
        rospy.Subscriber(str("/cube2/pose"), PoseWithCovarianceStamped, self.com_callback, queue_size=1)
        self.cube2 = np.array([0, 0])

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

        self.wheel_base = 0.10082
        self.wheel_radius = 0.01905

        self.landmark = {}
        self.landmark[0.0] = np.array([4.0, 0.0])       # for square
        self.landmark[1.0] = np.array([2.5, 4.0])
        self.landmark[2.0] = np.array([-1.5, 2.5])
        self.landmark[3.0] = np.array([0.0, -1.5])
        self.landmark[10.0] = np.array([1.0, 0.0])      # back april tag
        self.landmark[11.0] = np.array([1.0, 0.0])      # side april tag

        # Initialize kalman filter to None
        self.kalman = None
        self.last_update_time = rospy.get_time()
        self.predict_time = rospy.get_time()
        self.predict_rate = 5       # EKF Prediction rate (Hz)
        self.err_threshold = 0.2

    def control_callback(self, data):
        """ Receive control inputs and convert it to linear velocity (m/s)
        :data: JointState message from /cube/joint_states topic
        """

        # Initialize EKF first time when first receive control input
        if self.kalman is None:
            self.kalman = RobotEKF(wheelbase=self.wheel_base)

        angular_velocity = np.array([data.velocity[0], data.velocity[1]])

        # Convert angular velocity to linear velocity then set control_inputs accordingly
        self.control_inputs = angular_velocity * self.wheel_radius
    
    def com_callback(self, data):
        """ Receive other cube pose
        :data: PoseWithCovarianceStamped() message of other cube pose
        """

        cube2_pose = data.pose.pose.position
        self.cube2 = np.array([cube2_pose.x, cube2_pose.y])
        self.landmark[10.0] = self.cube2
        self.landmark[11.0] = self.cube2

    def tag_callback(self, data):
        """ Update step of EKF when AprilTag detected
        :data: Float32MultiArray from topic /cube/apriltag [ID, distance(m), bearing(rad)]
        """
        if self.kalman is None:
            return False
        
        prev_pose = self.kalman.pose    
        dt_pred = rospy.get_time() - self.predict_time
        dt = 0

        # Run prediction step at (self.predict_rate) Hz
        if dt_pred > (1 / self.predict_rate) :
            dt = rospy.get_time() - self.last_update_time
            
            # Prediction step
            self.kalman.predict(self.control_inputs, dt)
            self.predict_time = rospy.get_time()
            self.last_update_time = rospy.get_time()

            # Calculates error between EKF estimates and ground-truth
            self.calc_err(True, prev_pose)

            # Publishes kalman path
            self.pub_kalman()

        # Run Update Step
        if data.data:
            tag_id, distance, yaw = data.data[0], data.data[1], data.data[2]

            # If didn't run prediction step, run it
            if dt_pred < (1 / self.predict_rate):
                dt = rospy.get_time() - self.last_update_time
            u = self.control_inputs  #np.array([0, 0])        #self.control_inputs     
            z = np.array([[distance], [yaw]])
            pose2 = self.landmark[tag_id]   

            # Reset prediction + update step
            self.kalman.predict(u, dt)
            self.kalman.update(z, pose2, dt)
            self.last_update_time = rospy.get_time()

            # Calculates error between EKF estimates and ground-truth
            self.calc_err(False, prev_pose)

            # Publishes kalman path
            self.pub_kalman()

        return True

    def odom_sub(self, data):
        """ Publish odom path
        :data: Odometry message from /cube/diff_drive_controller/odom topic
        """
        # Set header
        self.odom_path.header = data.header

        # Set pose
        pose = PoseStamped()
        pose.header = data.header
        pose.pose = data.pose.pose

        # Publish 
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
        
        # Set header
        self.true_path.header.frame_id = "odom"
        self.true_path.header.seq += 1
        self.true_path.header.stamp = rospy.Time().now()

        pose = PoseStamped()
        pose.header = self.true_path.header
        # Set pose
        try:
            pose.pose = data.pose[self.link_id]
        except IndexError:
            for i, link in enumerate(data.name):
                if link == str(self.cube + "::" + self.cube + "_dummy"):
                    self.link_id = i
            pose.pose = data.pose[self.link_id]

        # Publish
        self.true_path.poses.append(pose)
        self.true_path_pub.publish(self.true_path)

    def pub_kalman(self):
        """ Publish EKF path and pose
        """
        quat = quaternion_from_euler(0.0, 0.0, self.kalman.pose[2])

        # Publish EKF pose
        pose_stamped = PoseWithCovarianceStamped()
        pose_stamped.header.frame_id = "odom"
        pose_stamped.header.seq += 1
        pose_stamped.header.stamp = rospy.Time().now()
        pose_stamped.pose.pose.position.x = self.kalman.pose[0, 0]
        pose_stamped.pose.pose.position.y = self.kalman.pose[1, 0]
        pose_stamped.pose.pose.position.z = 0.0
        pose_stamped.pose.pose.orientation.x = quat[0]
        pose_stamped.pose.pose.orientation.y = quat[1]
        pose_stamped.pose.pose.orientation.z = quat[2]
        pose_stamped.pose.pose.orientation.w = quat[3]
        pose_stamped.pose.covariance[0] = self.kalman.P[0,0]
        pose_stamped.pose.covariance[7] = self.kalman.P[1,1]
        pose_stamped.pose.covariance[35] = self.kalman.P[2,2]
        self.pose_pub.publish(pose_stamped)

        # Publish EKF path
        path_pose = PoseStamped()
        path_pose.header = pose_stamped.header
        path_pose.pose = pose_stamped.pose.pose
        self.kalman_path.header = pose_stamped.header
        self.kalman_path.poses.append(path_pose)
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
        
        # If error is higher than threshold, reset it accordingly
        if err.data > self.err_threshold:
            # rospy.loginfo("WHOOPSIE %s", self.cube)
            # Reset it to latest odom reading, taking into account transformation with world frame
            x = self.odom_path.poses[-1].pose.position.x + self.init_x
            y = self.odom_path.poses[-1].pose.position.y + self.init_y
            tx = self.odom_path.poses[-1].pose.orientation.x
            ty = self.odom_path.poses[-1].pose.orientation.y
            tz = self.odom_path.poses[-1].pose.orientation.z
            tw = self.odom_path.poses[-1].pose.orientation.w
            bearing = euler_from_quaternion([tx, ty, tz, tw])[2]
            self.kalman.pose = np.array([[x],[y],[bearing]])
            self.kalman.subs[self.kalman.x_x] = x
            self.kalman.subs[self.kalman.x_y] = y
            self.kalman.subs[self.kalman.x_theta] = bearing

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