#!/usr/bin/python3
import rospy
from time import sleep
from std_msgs.msg import Float32MultiArray
from gpiozero import Device, PhaseEnableMotor, RotaryEncoder
from gpiozero.pins.pigpio import PiGPIOFactory
from math import pi
from threading import Lock

motor_left = None
motor_right = None
encoder_left = None
encoder_right = None

lock = Lock()

desired_vel = None
curr_vel = [0.0, 0.0]
prev_error = (0.0, 0.0)
area = [0.0, 0.0]
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
    
def speed_controller(delta_t):
    global desired_vel
    global curr_vel
    global prev_error
    global area
    global curr_eff

    p = 0.3
    i = 0.5
    d = 0

    desired_vl, desired_vr = desired_vel

    left_eff = curr_eff[0]
    right_eff = curr_eff[1]

    curr_error = (desired_vl - curr_vel[0], desired_vr - curr_vel[1])

    with lock:
        area[0] = area[0] + ( 0.5 * (curr_error[0] + prev_error[0]) * delta_t)
        area[1] = area[1] + ( 0.5 * (curr_error[1] + prev_error[1]) * delta_t)

    l_proportion = curr_error[0] * p
    l_integral = area[0] * i
    l_derivative = ((curr_error[0] - prev_error[0]) / delta_t) * d

    r_proportion = curr_error[1] * p
    r_integral = area[1] * i
    r_derivative = ((curr_error[1] - prev_error[1]) / delta_t) * d

    left_eff = left_eff + l_proportion + l_integral + l_derivative
    right_eff = right_eff + r_proportion + r_integral + r_derivative

    with lock:
        curr_eff = (left_eff, right_eff)
        prev_error = curr_error

    drive_one_wheel(left_eff, True)
    drive_one_wheel(right_eff, False)

def update_desired(desired : Float32MultiArray):
    global desired_vel
    global prev_error
    global area
    global curr_vel
    linear, angular = desired.data
    
    l = 0.101 # meters
    desired_vl = linear - ((angular * l)/2)
    desired_vr = linear + ((angular * l)/2)

    with lock:
        desired_vel = (desired_vl, desired_vr)
        prev_error = (desired_vel[0] - curr_vel[0], desired_vel[1] - curr_vel[1])
        area = [0.0, 0.0]

if __name__ == '__main__':
    delta_t = 0.01
    rospy.init_node('locomotion')
    init()
    rospy.Subscriber("robot_twist", Float32MultiArray, update_desired)

    tolerance = 0.01

    prev_dist = get_distance()
    while not rospy.is_shutdown(): 
        sleep(delta_t)
        curr_dist = get_distance()

        wheel_vel = calc_wheel_vel(prev_dist, curr_dist, delta_t)
        prev_dist = curr_dist
        curr_vel = wheel_vel

        vel_pub.publish(Float32MultiArray(data=wheel_vel))

        if (abs(prev_error[0]) > tolerance or abs(prev_error[1]) > tolerance) and desired_vel != None:
            speed_controller(delta_t)
        else:
            desired_vel = None
        