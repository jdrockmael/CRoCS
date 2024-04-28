#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
import time
import math
import numpy as np
import math
from gazebo_msgs.msg import LinkStates
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int64, Int64MultiArray

# Convert CBS coordinates to real world coordinates and vice versa. 39.37 for inches and meter conversion. 15 is for robot radius 3in*2 + 4.5in*2 cspace
def path2world(path_pos):
    return (float(path_pos) * 15.0 / 39.37)

def world2path(world_pos):
    return (float(world_pos) * 39.37 / 15.0)

# Normalize angle between [-pi, pi]S
def normalize(angle):
    angle = angle % (2 * np.pi)
    if angle > np.pi:             
        angle -= 2 * np.pi
    return angle

class Main():
    def __init__(self):
        self.cube_id = rospy.get_param('drive/robot_name')       
        self.cube_id = int(self.cube_id[-1])
        self.vel_pub = rospy.Publisher(str("/cube" + str(self.cube_id) + "/diff_drive_controller/cmd_vel"), Twist, queue_size=10)

        self.path_state_pub = rospy.Publisher("/path_state", Int64MultiArray, queue_size=1)
        self.path_step_pub = rospy.Publisher("/path_step", Int64, queue_size=1)

        rospy.Subscriber("/path_state", Int64MultiArray, self.path_state_sub, queue_size=1)
        rospy.Subscriber("/path_step", Int64, self.path_step_sub, queue_size=1)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.true_sub, queue_size=1)

        self.link_id = 0
        self.d = 0.039624
        self.path_state = [0] * 7
        self.time_step = -1

        self.turn_speed = math.pi/8
        self.forward_speed = 0.2
        self.slow = self.cube_id / 10       # Delay for cube state publishing to preventr overlaps

        self.rate = rospy.Rate(100)
                
        time.sleep(5)

        if rospy.has_param("/cbs_output"):
            self.cube_paths = rospy.get_param("/cbs_output/schedule")
            self.path = self.cube_paths[str("cube" + str(self.cube_id))]

        rospy.spin()

    def path_state_sub(self, data):
        """ Subscriber for the current state of all robots
        :data: Int64Arr, len = # of cubes in the system, 0 for beginning of time step, 1 for finished turning, 2 for finished step, 
        3 for go forward and lock together, 4 for finished locking, 5 for crossing the gap, 6 for finished crossing, 7 for exploring
        """
        self.path_state = list(data.data)

        if self.path_state.count(3) == 7:
            self.locking()
            
        if self.path_state.count(5) == 7:
            self.drive(2.5, 0.1)

            time.sleep(self.slow)
            # Signal finished crossing gap
            if self.cube_id == 1:
                self.pub_state(6, True)

        if self.path_state.count(7) == 7:
            time.sleep(self.cube_id / 2)

            detach_dist = float(self.cube_id/7)
            self.explore(detach_dist)

    def locking(self):
        self.turn(- self.heading)

        # fuck this shite, could make it dynamic w apriltag reading
        if self.cube_id < 6:
            self.drive(0.24 * abs(self.cube_id-7), 0.05)
        elif self.cube_id < 5:
            self.drive(0.245 * abs(self.cube_id-7), 0.05)
        elif self.cube_id < 4:
            self.drive(0.248 * abs(self.cube_id-7), 0.05)
        else:
            self.drive(0.25 * abs(self.cube_id-7), 0.05)

        time.sleep(self.slow)

        # Signal finished locking
        if self.cube_id == 1:
            self.pub_state(4, True)
    
    def explore(self, dist):
        """ After detaching, go around and "explore"
        :dist: Distance in m to go forward after detaching
        """

        turn_angle = np.pi/2
        if (self.cube_id % 2) == 0:
            turn_angle = -turn_angle
        self.drive(dist, 0.1)
        self.turn(turn_angle)
        self.drive(dist, 0.1)
            

    def path_step_sub(self, data):
        """ Subscriber for the current time step of the path 
        :data: Int64 time step
        """
        self.time_step = data.data

        # Get goal position at current time step then drive towards it
        if self.time_step < len(self.path):
            step = self.path[self.time_step]
            rospy.loginfo("%s TIME %s CURR (%s, %s) GOAL (%s, %s)", str(self.cube_id), step['t'], world2path(self.pos[0]), world2path(self.pos[1]), step['x'], step['y'])

            goal = np.array([path2world(step['x']), path2world(step['y'])])
            self.drive_to_point(goal)

        # If finished with the last time step, turn towards the gap/0 heading
        if self.time_step+1 == len(self.path):
            self.turn(-self.heading)

        time.sleep(self.slow)

        # Update path state to finished time step state (2)
        self.pub_state(2, False)

    def drive_to_point(self, dest):
        """ Turns and drives to destinated goal location
        :dest: goal coordinate in meters (x, y)
        """
        self.dest = dest
        delta_x, delta_y = self.dest[0] - self.pos[0], self.dest[1] - self.pos[1]

        # Calculate angle needed to turn towards goal position
        turning_angle = 0
        if abs(delta_x) > 0.02 or abs(delta_y) > 0.02:
            turning_angle = math.atan2(delta_y, delta_x) - self.heading
        turning_angle = normalize(turning_angle)

        rospy.loginfo("%s  HEADING %s TURNING_ANGLE %s", str(self.cube_id), round(self.heading,5), round(turning_angle, 5))
        self.turn(turning_angle)
        
        # Calculate the distance forward to travel after turning to take offset wheel center into account
        delta_x, delta_y = self.dest[0] - self.pos[0], self.dest[1] - self.pos[1]
        distance = math.sqrt(delta_x**2 + delta_y**2)

        # Correct itself before turning to account for offset wheelbase
        if self.time_step == (len(self.path) - 1):
            if self.heading > 0:
                distance -= self.d 
            else:
                distance += self.d

        # Wait until all cubes finished turning before start driving forward
        while self.path_state.count(1) + self.path_state.count(2) < len(self.cube_paths):
            time.sleep(0.01)

        if self.time_step+1 == len(self.path):
            distance -= self.d

        self.drive(distance, self.forward_speed)
            
    def drive(self, distance, speed):
        """ Drive the robot forward for the given distance at the given speed
        :distance: distance to travel in meters
        :speed: linear velocity in m/s
        """
        init_x, init_y = self.pos[0], self.pos[1]

        # Distance travelled
        dist = math.sqrt((self.pos[0] - init_x)**2 + (self.pos[1] - init_y)**2)

        while dist < distance:
            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = 0
            self.vel_pub.publish(msg)
            dist = np.sqrt((self.pos[0] - init_x)**2 + (self.pos[1] - init_y)**2)
            self.rate.sleep()

        # Stop driving
        self.stop()

        # Variable wait to avoid overlapping between bots
        time.sleep(self.slow)

    def turn(self, angle):
        """ Turn the robot at the given angle and speed
        :angle: angle to turn [-pi, pi]. Positive to turn left, negative to turn right
        :speed: angular speed to turn in rad/s
        """
        # Get target heading by adding the turning angle with the current heading then normalize it
        target = normalize(angle + self.heading)

        while abs(self.heading - target) > 0.02:
            msg = Twist()
            msg.linear.x = 0
            msg.angular.z = self.turn_speed * np.sign(angle)
            self.vel_pub.publish(msg)
            self.rate.sleep()

        # Stop turning
        self.stop()

        # Variable wait to avoid overlapping between bots
        time.sleep(self.slow)

        # Update path state to finished turning state (1)
        self.pub_state(1, False)
    
    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0  
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)
    
    def pub_state(self, state, fillAll):
        new_state = Int64MultiArray()

        if fillAll:
            new_state.data = [state] * len(self.cube_paths)
        else:
            curr_state = self.path_state
            curr_state[self.cube_id - 1] = state
            new_state.data = curr_state

        self.path_state_pub.publish(new_state)

    def true_sub(self, data):
        """ Publish ground-truth gazebo path 
        :data: LinkStates message from /gazebo/link_states
        """
        # Find index number of cube link
        if self.link_id == 0:
            for i, link in enumerate(data.name):
                if link == str("cube" + str(self.cube_id) + "::cube" + str(self.cube_id) + "_dummy"):
                    self.link_id = i

        # Set pose
        try:
            pose = data.pose[self.link_id]

            self.pos = np.array([pose.position.x, pose.position.y])
            self.heading = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]

        except IndexError:
            for i, link in enumerate(data.name):
                if link == str("cube" + str(self.cube_id) + "::cube" + str(self.cube_id) + "_dummy"):
                    self.link_id = i
            pose = data.pose[self.link_id]
            self.pos = np.array([pose.position.x, pose.position.y])        
    
if __name__ == '__main__':
    rospy.init_node('drive', anonymous=False)
    Main()


