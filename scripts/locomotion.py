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
curr_eff = (0.0, 0.0)

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

def vel_to_eff(desired_vel):
    return 1.42 * pow(desired_vel, 0.683)

def drive_one_wheel(pwd, is_left):
    if pwd > 1:
        pwd = 1
    elif pwd < -1:
        pwd = -1
    elif pwd < 0.18 and pwd > -0.18:
        pwd = 0

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
    global curr_eff
    linear, angular = twist.data
    
    l = 0.101 # meters
    desired_vl = linear - ((angular * l)/2)
    desired_vr = linear + ((angular * l)/2)

    tolerance = 0.01
    delta_t = 0.01

    p = 0.3
    i = 1
    d = 1

    left_eff = curr_eff[0]
    right_eff = curr_eff[1]

    while abs(desired_vl - curr_vel[0]) > tolerance or abs(desired_vr - curr_vel[1]) > tolerance:
        l_proportion = (desired_vl - curr_vel[0]) * p
        l_integral = ((desired_vl - curr_vel[0]) * delta_t) * i
        l_derivative = ((desired_vl - curr_vel[0]) / delta_t) * d

        r_proportion = (desired_vr - curr_vel[1]) * p
        r_integral = ((desired_vr - curr_vel[1]) * delta_t) * i
        r_derivative = ((desired_vr - curr_vel[1]) / delta_t) * d

        left_eff = left_eff + l_proportion + l_integral + l_derivative
        right_eff = right_eff + r_proportion + r_integral + r_derivative

        curr_eff = (left_eff, right_eff)
        rospy.logerr(curr_eff)

        drive_one_wheel(left_eff, True)
        drive_one_wheel(right_eff, False)

        sleep(delta_t)

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
        