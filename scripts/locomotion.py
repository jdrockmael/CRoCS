#!/usr/bin/python3
import rospy
from time import sleep
from std_msgs.msg import Float32MultiArray, Bool
from gpiozero import Device, PhaseEnableMotor, RotaryEncoder, Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from math import pi, sin, cos
from threading import Thread, Lock

# motor objects used to control the motors and servos
motor_left = None
motor_right = None
encoder_left = None
encoder_right = None
servo1 = None
servo2 = None

# desired velocity and current used for speed controller
desired_vel = None
curr_vel = (0.0, 0.0)

# lock used to avoid mutex error but this is probably not needed
desired_lock = Lock()
vel_lock = Lock()

pose_pub = rospy.Publisher('curr_pose', Float32MultiArray, queue_size= 1)

# creates the motor objects used to drive the wheels and servo
def init():
    global motor_left 
    global motor_right
    global encoder_left
    global encoder_right
    global servo1
    global servo2

    Device.pin_factory = PiGPIOFactory()

    motor_left = PhaseEnableMotor(24, 23)
    motor_right = PhaseEnableMotor(21, 20)

    encoder_left = RotaryEncoder(27, 22, max_steps = 256000)
    encoder_right = RotaryEncoder(5, 6, max_steps = 256000)

    servo1 = Servo(25)
    servo2 = Servo(8)

    motor_left.stop()
    motor_right.stop()

# stops the motors and sets the desired vel to None
def kill_motors(turn_off : Bool):
    global desired_vel
    global curr_vel

    if turn_off.data:
        motor_left.stop()
        motor_right.stop()

        with desired_lock:
            desired_vel = None

# opens the gripper is given true, closes if false
def open_grip(want_open : Bool):
    if want_open.data:
        servo1.value = -1
        servo2.value = 1
        sleep(2)
    else:
        servo1.value = 1
        servo2.value = -1
        sleep(2)

# gets the distance that the wheels traveled 
def get_distance():
    tick_per_rev = 128.0
    r_of_wheel = 0.021225 # in meters
    left = (encoder_left.steps / tick_per_rev) * (2.0 * pi * r_of_wheel)
    right = (-encoder_right.steps / tick_per_rev) * (2.0 * pi * r_of_wheel)

    return (left, right)

# Using the previous, current, and the difference in time calculates the velocity of wheels
def calc_wheel_vel(prev_wheel_dis, curr_wheel_dis, delta_t):
    delta_l = curr_wheel_dis[0] - prev_wheel_dis[0]
    delta_r = curr_wheel_dis[1] - prev_wheel_dis[1]

    left_vel = delta_l / delta_t
    right_vel = delta_r / delta_t

    return (left_vel, right_vel)

# Drives one with power and a boolean; if true, it drives left, if false it drive right
def drive_one_wheel(pwd, is_left):
    if pwd > 1:
        pwd = 1
    elif pwd < -1:
        pwd = -1
    elif pwd < 0.2 and pwd > -0.2:
        # logic needs to be changed in later work so that wheels are able to drive at lower powers
        pwd = 0

    if pwd >= 0 and is_left:
        motor_left.forward(pwd)
    elif pwd < 0 and is_left:
        motor_left.backward(-pwd)
    elif pwd >= 0 and not is_left:
        motor_right.backward(pwd)
    else:
        motor_right.forward(-pwd)

# powers the wheels using direct effort values (not really used)
def set_eff(efforts : Float32MultiArray):
    l, r = efforts.data
    drive_one_wheel(l, True)
    drive_one_wheel(r, False)

# using the wheel velocity, previous pose, and the difference in time,
# calculates the new pose of the robot
def calc_fk(wheel_vel, prev_pose, delta_t):
    l = 0.101 # meters
    vl, vr = wheel_vel

    omega = (vr - vl) / l
    vel = (vr + vl) / 2

    x, y, theta = prev_pose[0], prev_pose[1], prev_pose[2]

    new_x = x + delta_t * vel * cos(theta)
    new_y = y + delta_t * vel * sin(theta)
    new_theta = theta + delta_t * omega

    temp = new_theta
    if temp == 0.0:
        temp += 1
    soh = temp/abs(temp)
    new_theta = new_theta % (soh * 2 * pi)
    if new_theta < 0:
        new_theta = 2*pi + new_theta

    return [new_x, new_y, new_theta]

# changes the global desired pose
def update_desired(desired : Float32MultiArray):
    global desired_vel
    linear, angular = desired.data
    
    l = 0.101 # meters
    desired_vl = linear - ((angular * l)/2)
    desired_vr = linear + ((angular * l)/2)

    with desired_lock:
        desired_vel = (desired_vl, desired_vr)

# updates the global current velocity variable (bad name for funciton)
def monitor_pose():
    global curr_vel
    delta_t = 0.05

    is_moving = True
    prev_dist = get_distance()
    prev_pose = [0.0, 0.0, 0.0]
    while not rospy.is_shutdown(): 
        sleep(delta_t)
        curr_dist = get_distance()

        if prev_dist != curr_dist or is_moving:

            if prev_dist != curr_dist:
                is_moving = True
            else:
                is_moving = False

            wheel_vel = calc_wheel_vel(prev_dist, curr_dist, delta_t)
            prev_dist = curr_dist

            with vel_lock:
                curr_vel = wheel_vel

            pose = calc_fk(wheel_vel, prev_pose, delta_t)
            prev_pose = pose
            pose_pub.publish(Float32MultiArray(data=pose))        

# by changing the global desired variable we can control the speed
# of the wheels with this function
def speed_controller():
    global desired_vel
    tolerance = 0.05
    delta_t = 0.05

    p = 0.35
    i = 0.2

    curr_eff = (0.0, 0.0)
    area = (0.0, 0.0)
    prev_desired = None

    while not rospy.is_shutdown():
        if desired_vel != None:
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

                drive_one_wheel(left_eff, True)
                drive_one_wheel(right_eff, False)
            else:
                with desired_lock:
                    desired_vel = None

            sleep(delta_t)

if __name__ == '__main__':
    rospy.init_node('locomotion')
    init()
    rospy.Subscriber("robot_twist", Float32MultiArray, update_desired)
    rospy.Subscriber("kill_motors", Bool, kill_motors)
    rospy.Subscriber("gripper_control", Bool, open_grip)
    rospy.Subscriber("set_effort", Float32MultiArray, set_eff)

    # starts a thread for calculated the speed of the wheels
    vel_thread = Thread(target=monitor_pose)
    vel_thread.start()

    # runs the speed controller function
    speed_controller()
    vel_thread.join()
    