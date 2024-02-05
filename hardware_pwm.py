from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo
from time import sleep

factory = PiGPIOFactory('192.168.1.169')
servo = Servo(25, pin_factory=factory)

servo.min()
sleep(1)
servo.max()
sleep(1)
servo.mid()
sleep(1)