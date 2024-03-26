#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
import time
import math
from std_msgs.msg import Bool
import numpy as np

import yaml
import argparse
import math
from gazebo_msgs.msg import ModelStates, LinkStates
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int64, Int64MultiArray

def path2world(path_pos):
    return (float(path_pos) * 12.0 / 39.37)

def world2path(world_pos):
    return (float(world_pos) * 39.37 / 12.0)

class Main():
    def __init__(self):
        self.cube = rospy.get_param('square/robot_name')       
        self.cube_id = int(self.cube[-1])
        self.x_init = rospy.get_param('square/x_pos')   
        self.vel_pub = rospy.Publisher(str("/" + self.cube + "/diff_drive_controller/cmd_vel"), Twist, queue_size=10)
        square_pub = rospy.Publisher(str("/" + self.cube + "/square"), Bool, queue_size=1)

        self.path_state_pub = rospy.Publisher("/path_state", Int64MultiArray, queue_size=1)
        self.path_step_pub = rospy.Publisher("/path_step", Int64, queue_size=1)
        rospy.Subscriber("/path_state", Int64MultiArray, self.path_state_sub, queue_size=1)
        rospy.Subscriber("/path_step", Int64, self.path_step_sub, queue_size=1)

        rospy.Subscriber("/gazebo/link_states", LinkStates, self.true_sub, queue_size=1)

        self.link_id = 0
        self.longest = False
        self.d = 0.039624
        self.path_state = [0] * 7
        self.time_step = -1
        
        # self.drive_to_point([0,0],[0,1])
        
        time.sleep(2)
      
        # rospy.sleep(rospy.Duration(secs=0.01)) # give time to process inbound msgs
        if rospy.has_param("/cbs_output"):
            self.cube_path = rospy.get_param(str("/cbs_output/schedule/" + self.cube))
            self.paths = [None] * 7
            for i, path in enumerate(self.paths):
                self.paths[i] = rospy.get_param("/cbs_output/schedule/cube" + str(i+1))
                # rospy.loginfo(i+1)
                # rospy.loginfo("LENGTH")
                # rospy.loginfo(len(self.paths[i]))

            if (max(self.paths, key=len) == self.cube_path):
                self.longest = True
                time.sleep(5)
                self.path_step_pub.publish(0)

        rospy.spin()

    def path_state_sub(self, data):
        self.path_state = list(data.data)
        # rospy.loginfo(self.path_state)
        if self.longest and self.path_state[0] and self.path_state[1]:
            rospy.loginfo("NEXT STEP")
            new_state = Int64MultiArray()
            new_state.data = [0] * 7
            self.path_state_pub.publish(new_state)
            self.path_step_pub.publish(self.time_step+1)

    def path_step_sub(self, data):
        path = self.paths[self.cube_id - 1]
        self.time_step = data.data

        if self.time_step < len(path):
            step = path[self.time_step]
            rospy.loginfo("CUBE %s TIME %s CURR (%s, %s) GOAL (%s, %s)", self.cube, step['t'], world2path(self.pos[0]), world2path(self.pos[1]), step['x'], step['y'])
            # rospy.loginfo("TIME %s", step['t'])
            # rospy.loginfo("X %s", step['x'])
            # rospy.loginfo("Y %s", step['y'])
            goal = np.array([path2world(step['x']), path2world(step['y'])])
            self.drive_to_point(goal)

        time.sleep(1/self.cube_id)

        curr_state = self.path_state
        curr_state[self.cube_id - 1] = 1

        new_state = Int64MultiArray()
        new_state.data = curr_state
        self.path_state_pub.publish(new_state)

    def true_sub(self, data):
        """ Publish ground-truth gazebo path 
        :data: LinkStates message from /gazebo/link_states
        """
        # Find index number of cube link
        if self.link_id == 0:
            for i, link in enumerate(data.name):
                if link == str(self.cube + "::" + self.cube + "_dummy"):
                    self.link_id = i

        # Set pose
        try:
            pose = data.pose[self.link_id]

            self.pos = np.array([pose.position.x, pose.position.y])
            self.heading = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]

        except IndexError:
            for i, link in enumerate(data.name):
                if link == str(self.cube + "::" + self.cube + "_dummy"):
                    self.link_id = i
            pose = data.pose[self.link_id]
            self.pos = np.array([pose.position.x, pose.position.y])

            
    def drive(self, distance, speed):
        # Drive foward in m/s
        rate = rospy.Rate(100)
        init_x, init_y = self.pos[0], self.pos[1]

        # Distance travelled
        dist = math.sqrt((self.pos[0] - init_x)**2 + (self.pos[1] - init_y)**2)

        while dist < distance:
            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = 0
            self.vel_pub.publish(msg)
            dist = np.sqrt((self.pos[0] - init_x)**2 + (self.pos[1] - init_y)**2)
            # rospy.loginfo(dist)
            rate.sleep()

        msg = Twist()
        msg.linear.x = 0.0  
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)

        time.sleep(0.5)

    def turn(self, angle, speed):
        # Speed in rad/s

        # Get how many seconds to turn by pi/2 / rad/s
        target = angle + self.heading
        target = target % (2 * np.pi)
        if target > np.pi:             
            target -= 2 * np.pi
        rate = rospy.Rate(100)

        while abs(self.heading - target) > 0.05:
            msg = Twist()
            msg.linear.x = 0
            msg.angular.z = speed * np.sign(angle)
            self.vel_pub.publish(msg)
            rate.sleep()

        msg = Twist()
        msg.linear.x = 0.0  
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)

        time.sleep(0.5)
        
    def drive_to_point(self, dest):
        delta_x, delta_y = dest[0] - self.pos[0], dest[1] - self.pos[1]
        turn_speed = math.pi/16
        forward_speed = 0.05

        turning_angle = 0
        if abs(delta_x) > 0.001 or abs(delta_y) > 0.001:
            turning_angle = math.atan2(delta_y, delta_x) - self.heading

        turning_angle2 = turning_angle % (2 * np.pi)
        if turning_angle2 > np.pi:             
            turning_angle2 -= 2 * np.pi

        rospy.loginfo("%s  ATAN 2 %s HEADING %s PREV_TURNING %s TURNING_ANGLE %s", self.cube, round(math.atan2(delta_y, delta_x),  3), round(self.heading,5), round(turning_angle, 5), round(turning_angle2, 5))
        self.turn(turning_angle2, turn_speed)

        delta_x, delta_y = dest[0] - self.pos[0], dest[1] - self.pos[1]
        distance = math.sqrt(delta_x**2 + delta_y**2)

        # rospy.loginfo("DRIVING DISTANCE %s", distance)
        self.drive(distance, forward_speed)

    def square(self, x_init):
        turn_speed = math.pi / 16
        turn_angle = math.pi/2
        forward = 2.5
        forward_speed = 0.03
        self.drive(2.5 - x_init, forward_speed)
        self.turn(turn_angle, turn_speed)
        self.drive(forward, forward_speed)
        self.turn(turn_angle, turn_speed)
        self.drive(forward, forward_speed)
        self.turn(turn_angle, turn_speed)
        self.drive(forward, forward_speed)
        self.turn(turn_angle, turn_speed)
        
    
if __name__ == '__main__':
    rospy.init_node('square', anonymous=False)
    Main()

    done = Bool(data=True)
    if done.data:
        rospy.loginfo(done)

