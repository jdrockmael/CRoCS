from gpiozero import Device, PhaseEnableMotor, RotaryEncoder
from gpiozero.pins.pigpio import PiGPIOFactory
import rospy
from std_msgs.msg import Float32MultiArray, String
from time import sleep

#forward ccw, backward cw

out_msgs_pub = rospy.Publisher('out_going_msgs', String, queue_size=10)

def init_motors():
    Device.pin_factory = PiGPIOFactory()

    motor1 = PhaseEnableMotor(24, 23)
    motor2 = PhaseEnableMotor(21, 20)

    motor1.stop()
    motor2.stop()
    sleep(1)

    encoder1 = RotaryEncoder(6, 5, max_steps = 256000)
    encoder2 = RotaryEncoder(22, 27, max_steps = 256000)

    return [(motor1, encoder1), (motor2, encoder2)]

def set_motor_pwd(motor : PhaseEnableMotor, power):
    if power > 1:
        power = 1
    elif power < -1:
        power = -1

    if power >= 0:
        motor.forward(power)
    else:
        motor.backward(power)

def control_loop(data : Float32MultiArray, tuple_of_motors):
    left, right = tuple_of_motors
    data = data.data

    heading_err = data[2] - 0.0
    distance_err = data[1] - 0.2 #m i think

    while abs(heading_err) > 1 or abs(distance_err) > 100:
        linear = distance_err
        angular_l = heading_err
        angular_r = -heading_err

        out_msgs_pub.publish("server left_effort=" + str(linear + angular_l) + "_right_effort=" + str(linear + angular_r))

        set_motor_pwd(left, linear + angular_l)
        set_motor_pwd(right, -(linear + angular_r)) # because the motor goes the other way
        sleep(0.005)

    left.stop()
    right.stop()

if __name__ == '__main__':
    rospy.init_node('pid')

    motor_list = init_motors()
    left, _ = motor_list[0]
    right, _ = motor_list[1]

    rospy.Subscriber("range", Float32MultiArray, control_loop, callback_args= (left, right))
    rospy.spin()
