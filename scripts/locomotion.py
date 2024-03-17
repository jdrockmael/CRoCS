#!/usr/bin/python3
import rospy
from time import sleep
from std_msgs.msg import Float32MultiArray
from gpiozero import Device, PhaseEnableMotor, RotaryEncoder
from gpiozero.pins.pigpio import PiGPIOFactory
from math import pi

motor_left = None
motor_right = None
encoder_left = None
encoder_right = None

curr_vel = (0.0, 0.0)

vel_pub = rospy.Publisher('wheel_vel', Float32MultiArray, queue_size= 1)

def init():
    global motor_left 
    global motor_right
    global encoder_left
    global encoder_right

    Device.pin_factory = PiGPIOFactory()

    motor_left = PhaseEnableMotor(24, 23)
    motor_right = PhaseEnableMotor(21, 20)

    encoder_left = RotaryEncoder(27, 22, max_steps = 256000)
    encoder_right = RotaryEncoder(5, 6, max_steps = 256000)

def drive_one_wheel(pwd, is_left):
    if pwd > 1:
        pwd = 1
    elif pwd < -1:
        pwd = -1

    if pwd >= 0 and is_left:
        motor_left.forward(pwd)
    elif pwd < 0 and is_left:
        motor_left.backward(-pwd)
    elif pwd >= 0 and not is_left:
        motor_right.backward(pwd)
    else:
        motor_right.forward(-pwd)
    
def drive(twist : Float32MultiArray):
    global curr_vel
    linear, angular = twist.data
    
    if linear != 0.0 or angular != 0.0:
        l = 0.101 # meters
        tolerance = 0.01

        desired_vl = linear - ((angular * l)/2)
        desired_vr = linear + ((angular * l)/2)

        while abs(desired_vl - curr_vel[0]) > tolerance or abs(desired_vr - curr_vel[1]) > tolerance:
            left_p = desired_vl - curr_vel[0]
            right_p = desired_vr - curr_vel[1]
            rospy.logerr((left_p, right_p))
            drive_one_wheel(left_p, True)
            drive_one_wheel(right_p, False)
    else:
        motor_left.stop()
        motor_right.stop()

def get_distance():
    tick_per_rev = 128.0
    r_of_wheel = 0.021225 # in meters
    left = (encoder_left.steps / tick_per_rev) * (2.0 * pi * r_of_wheel)
    right = (-encoder_right.steps / tick_per_rev) * (2.0 * pi * r_of_wheel)

    return (left, right)

def calc_wheel_vel(prev_wheel_dis, curr_wheel_dis, delta_t):
    delta_l = curr_wheel_dis[0] - prev_wheel_dis[0]
    delta_r = curr_wheel_dis[1] - prev_wheel_dis[1]

    left_vel = delta_l / delta_t
    right_vel = delta_r / delta_t

    return (left_vel, right_vel)

if __name__ == '__main__':
    delta_t = 0.3
    rospy.init_node('locomotion')
    init()
    rospy.Subscriber("robot_twist", Float32MultiArray, drive)

    prev_dist = get_distance()
    while not rospy.is_shutdown():
        sleep(delta_t)
        curr_dist = get_distance()

        wheel_vel = calc_wheel_vel(prev_dist, curr_dist, delta_t)
        prev_dist = curr_dist
        curr_vel = wheel_vel

        vel_pub.publish(Float32MultiArray(data=wheel_vel))
        