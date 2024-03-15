from gpiozero import Device, PhaseEnableMotor, RotaryEncoder
from gpiozero.pins.pigpio import PiGPIOFactory
#forward ccw, backward cw
from time import sleep

Device.pin_factory = PiGPIOFactory()

motor1 = PhaseEnableMotor(24, 23)
motor2 = PhaseEnableMotor(21, 20)

motor1.stop()
motor2.stop()
sleep(1)

encoder1 = RotaryEncoder(6, 5, max_steps = 256000)
encoder2 = RotaryEncoder(22, 27, max_steps = 256000)

motor1.forward(0.5)
motor2.backward(0.5)
sleep(0.01)

i = 0
while(i < 3000):
    print("1: ", encoder1.steps)
    print("2: ", encoder2.steps)
    sleep(0.0001)
    i+=1

'''
motor1.backward(0.5)
motor2.forward(0.5)
sleep(0.01)


j = 0
while(j < 5000):
    print("1: ", encoder1.steps)
    print("2: ", encoder2.steps)
    sleep(0.0001)
    j+=1
'''
sleep(0.0001)
motor1.stop()
motor2.stop()

