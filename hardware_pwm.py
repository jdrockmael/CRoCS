from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo

factory = PiGPIOFactory()
servo = Servo(25, pin_factory=factory)

servo.min()
servo.max()
servo.mid()