from gpiozero import PhaseEnableMotor
#import RPi.GPIO as GPIO
from time import sleep

servo = PhaseEnableMotor(26, 13)

for i in range(10):
    servo.forward(i)
    sleep(1)
    servo.backward(i)
    sleep(1)
    i+=1
