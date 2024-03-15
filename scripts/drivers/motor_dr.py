from gpiozero import Device, PhaseEnableMotor, RotaryEncoder
from gpiozero.pins.pigpio import PiGPIOFactory
from math import pi

class Motor():
    motor_left = None
    motor_right = None
    encoder_left = None
    encoder_right = None

    def __init__(self) -> None:
        global motor_left 
        global motor_right
        global encoder_left
        global encoder_right

        Device.pin_factory = PiGPIOFactory()

        motor_left = PhaseEnableMotor(24, 23)
        motor_right = PhaseEnableMotor(21, 20)

        encoder_left = RotaryEncoder(6, 5, max_steps = 256000)
        encoder_right = RotaryEncoder(22, 27, max_steps = 256000)

    def drive_one_wheel(self, pwd, is_left):
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
        
    def drive(self, left_p, right_p):
        self.drive_one_wheel(left_p, True)
        self.drive_one_wheel(right_p, False)

    def stop(self):
        motor_left.stop()
        motor_right.stop()

    def get_distance(self):
        tick_per_rev = 128.0
        r_of_wheel = 0.04245 # in meters
        left = (encoder_left.steps / tick_per_rev) * (2.0 * pi * r_of_wheel)
        right = (encoder_right.steps / tick_per_rev) * (2.0 * pi * r_of_wheel)

        return (left, right)
    
motor = Motor()