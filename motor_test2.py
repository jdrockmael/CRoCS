from gpiozero import Servo
import RPi.GPIO as GPIO
from time import sleep

servo = Servo(16)

while True:
    servo.mid()
    sleep(0.5)
    servo.max()
    sleep(0.5)
