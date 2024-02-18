from gpiozero import Device, PhaseEnableMotor, RotaryEncoder
from gpiozero.pins.native import NativeFactory
#forward ccw, backward cw
from time import sleep

Device.pin_factory = NativeFactory()

#motor1 = PhaseEnableMotor(26, 13)
#motor2 = PhaseEnableMotor(24, 23)

encoder1 = RotaryEncoder(5, 6)
encoder2 = RotaryEncoder(20, 21)

while(True):
    print(encoder1.value)
    print(encoder2.value)
