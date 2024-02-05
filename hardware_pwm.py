"""
from gpiozero.pins.pigpio import PiGPIOFactory
import pigpio
from gpiozero import Servo
from time import sleep

pi = pigpio.pi()
servo = pi.hardware_PWM(pi, 18, 800, 250000) # 800Hz 25% dutycycle

factory = PiGPIOFactory('192.168.1.169')
#servo = Servo(25, pin_factory=factory)

servo.min()
sleep(1)
servo.max()
sleep(1)
servo.mid()
sleep(1)
"""

import pigpio
from time import sleep 

servo = 18
 
pwm = pigpio.pi() 
pwm.set_mode(servo, pigpio.OUTPUT)
 
pwm.set_PWM_frequency(servo, 50)
pwm.set_PWM_range(servo, 20000) # 1,000,000 / 50 = 20,000us for 100% duty cycle

pwm.hardware_PWM(servo, 50, 2000)
sleep(10)

pwm.set_servo_pulsewidth(servo, 50)
sleep(1)