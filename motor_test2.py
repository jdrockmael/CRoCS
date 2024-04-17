from gpiozero import Device, PhaseEnableMotor, RotaryEncoder, Servo
from gpiozero.pins.pigpio import PiGPIOFactory
#forward ccw, backward cw
from time import sleep

Device.pin_factory = PiGPIOFactory()

motor_left = PhaseEnableMotor(24, 23)
motor_right = PhaseEnableMotor(21, 20)

encoder_left = RotaryEncoder(27, 22, max_steps = 256000)
encoder_right = RotaryEncoder(5, 6, max_steps = 256000)

servo1 = Servo(25)
servo2 = Servo(8)

motor_left.stop()
motor_right.stop()

# open
servo1.value = -1
servo2.value = 1
sleep(2)

# close
servo1.value = 1
servo2.value = -1
sleep(2)

motor_left.forward(0.5)
motor_right.backward(0.5)
sleep(0.01)

i = 0
while(i < 3000):
    print("left: ", encoder_left.steps)
    print("right: ", encoder_right.steps)
    sleep(0.0001)
    i+=1

sleep(0.0001)
motor_left.stop()
motor_right.stop()


