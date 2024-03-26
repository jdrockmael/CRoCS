#!/usr/bin/python3
import rospy
from time import sleep
from std_msgs.msg import Float32MultiArray
from gpiozero import Device, PhaseEnableMotor, RotaryEncoder
from gpiozero.pins.pigpio import PiGPIOFactory
from math import pi
from threading import Thread, Lock

motor_left = None
motor_right = None
encoder_left = None
encoder_right = None

desired_vel = (0.0, 0.0)
curr_vel = [0.0, 0.0]

desired_lock = Lock()
vel_lock = Lock()

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
    
def update_desired(desired : Float32MultiArray):
    global desired_vel
    linear, angular = desired.data
    
    l = 0.101 # meters
    desired_vl = linear - ((angular * l)/2)
    desired_vr = linear + ((angular * l)/2)

    with desired_lock:
        desired_vel = (desired_vl, desired_vr)

    what = "setting global to", desired_vel
    rospy.logerr(what)

def monitor_vel():
    global curr_vel
    delta_t = 0.01

    prev_dist = get_distance()
    while not rospy.is_shutdown(): 
        sleep(delta_t)
        curr_dist = get_distance()

        wheel_vel = calc_wheel_vel(prev_dist, curr_dist, delta_t)
        prev_dist = curr_dist

        with vel_lock:
            curr_vel = wheel_vel

        vel_pub.publish(Float32MultiArray(data=wheel_vel))
        
def speed_controller():
    global desired_vel
    tolerance = 0.05
    delta_t = 0.05

    p = 0.3
    i = 0.5

    curr_eff = (0.0, 0.0)
    area = (0.0, 0.0)
    prev_desired = (None, None)

    while not rospy.is_shutdown():
        if desired_vel != (None, None):
            curr_l, curr_r = curr_vel
            desired_l, desired_r = desired_vel
            curr_error = (desired_l - curr_l, desired_r - curr_r)

            if prev_desired != (desired_l, desired_r):
                area = (0.0, 0.0)
                prev_desired = (desired_l, desired_r)

            if abs(curr_error[0]) > tolerance or abs(curr_error[1]) > tolerance:
                area = (area[0] + curr_error[0] * delta_t, area[1] + curr_error[1] * delta_t)

                left_eff = curr_eff[0] + curr_error[0] * p + area[0] * i
                right_eff = curr_eff[1] + curr_error[1] * p + area[1] * i

                curr_eff = (left_eff, right_eff)

                eff_pub.publish(Float32MultiArray(data=[left_eff, right_eff]))
            else:
                desired_vel = (None, None)

            sleep(delta_t)

if __name__ == '__main__':
    rospy.init_node('locomotion')
    init()
    rospy.Subscriber("robot_twist", Float32MultiArray, update_desired)

    vel_thread = Thread(target=monitor_vel)
    vel_thread.start()

    speed_controller()