from gpiozero.pins.pigpio import PiGPIOFactory
import pigpio
from gpiozero import Servo
from time import sleep

servo = pigpio.pi.hardware_PWM(pigpio.pi, 18, 800, 250000) # 800Hz 25% dutycycle

factory = PiGPIOFactory('192.168.1.169')
#servo = Servo(25, pin_factory=factory)

servo.min()
sleep(1)
servo.max()
sleep(1)
servo.mid()
sleep(1)