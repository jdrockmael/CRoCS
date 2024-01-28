from gpiozero import PhaseEnableMotor
#import RPi.GPIO as GPIO
from time import sleep

motor1 = PhaseEnableMotor(26, 13)
motor2 = PhaseEnableMotor(6, 5)

for i in range(1,10):
    motor1.forward(i/10)
    sleep(2)
    motor2.forward(i/10)
    sleep(2)
    i+=1
