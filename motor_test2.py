from gpiozero import PhaseEnableMotor
#import RPi.GPIO as GPIO
from time import sleep

servo = PhaseEnableMotor(13)

while True:
    servo.forward()
    sleep(0.5)
    servo.backward()
    sleep(0.5)
