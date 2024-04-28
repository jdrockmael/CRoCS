from gpiozero import Device, PhaseEnableMotor, RotaryEncoder
from gpiozero.pins.pigpio import PiGPIOFactory    

Device.pin_factory = PiGPIOFactory()

motor_left = PhaseEnableMotor(24, 23)
motor_right = PhaseEnableMotor(21, 20)

encoder_left = RotaryEncoder(27, 22, max_steps = 256000)
encoder_right = RotaryEncoder(5, 6, max_steps = 256000)

motor_left.stop()
motor_right.stop()