from gpiozero import Device, PhaseEnableMotor, RotaryEncoder
from gpiozero.pins.pigpio import PiGPIOFactory
from math import pi

class Motor():
    motor_object = None
    encoder_object = None
    is_ccw_positive = None

    def __init__(self, phase, enable, encoder_pin_a, encoder_pin_b, pwd_multplier = 1) -> None:
        global motor_object
        global encoder_object
        global is_ccw_positive   # negative one means no and one means yes

        is_ccw_positive = pwd_multplier

        Device.pin_factory = PiGPIOFactory()

        motor_object = PhaseEnableMotor(phase, enable)

        encoder_object = RotaryEncoder(encoder_pin_a, encoder_pin_b, max_steps = 256000)

    def drive(pwd : int):
        pwd = pwd * is_ccw_positive

        if pwd > 1:
            pwd = 1
        elif pwd < -1:
            pwd = -1

        if pwd >= 0:
            motor_object.forward(pwd)
        else:
            motor_object.backward(-pwd)

    def stop():
        motor_object.stop()

    def get_distance():
        tick_per_rev = 128
        r_of_wheel = 0.02 # in meters

        return (encoder_object.steps / tick_per_rev) * (2 * pi * r_of_wheel)