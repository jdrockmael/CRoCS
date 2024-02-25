#!/usr/bin/python3
from gpiozero import Device, PhaseEnableMotor, RotaryEncoder
from gpiozero.pins.pigpio import PiGPIOFactory
import rospy
from std_msgs.msg import Float32MultiArray, String, Bool
from time import sleep

#forward ccw, backward cw

out_msgs_pub = rospy.Publisher('out_going_msgs', String, queue_size=10)

Device.pin_factory = PiGPIOFactory()

motor_left = PhaseEnableMotor(24, 23)
motor_right = PhaseEnableMotor(21, 20)

motor_left.stop()
motor_right.stop()
sleep(1)

encoder_left = RotaryEncoder(6, 5, max_steps = 256000)
encoder_right = RotaryEncoder(22, 27, max_steps = 256000)

def bound_pwd(power):
    if power > 1:
        power = 1
    elif power < -1:
        power = -1

    return power

d_prior_err = 0
r_prior_err = 0
l_prior_err = 0

def control_loop(data : Float32MultiArray):
    global d_prior_err
    global r_prior_err
    global l_prior_err
    data = data.data
    delta_t = 0.001

    if len(data) != 0:
        heading_err = data[2] - 0.0
        distance_err = data[1] - 0.2 #m i think
        
        linear = distance_err * 10 + ((distance_err - d_prior_err)/delta_t) * 3
        angular_l = heading_err * 3 + ((angular_l - l_prior_err)/delta_t) * 3
        angular_r = -heading_err * 3 + ((angular_r - r_prior_err)/delta_t) * 3
        
        d_prior_err = distance_err
        r_prior_err = angular_r
        l_prior_err = angular_l

        l_eff = bound_pwd(linear + angular_l)
        r_eff = bound_pwd(linear + angular_r)

        out_msgs_pub.publish("server left_effort=" + str(l_eff) + "_right_effort=" + str(r_eff))

        if l_eff >= 0:
            motor_left.forward(l_eff)
        else:
            motor_left.backward(-l_eff)

        if r_eff >= 0:
            motor_right.backward(r_eff)
        else:
            motor_right.forward(-r_eff)

        sleep(delta_t)
    else:
        d_prior_err = 0
        r_prior_err = 0
        l_prior_err = 0
        motor_left.stop()
        motor_right.stop()

if __name__ == '__main__':
    rospy.init_node('pid')

    rospy.Subscriber("range", Float32MultiArray, control_loop)

    rospy.spin()
