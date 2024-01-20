from gpiozero import Servo
from time import sleep

servo = Servo(21)

while True:
    servo.mid()
    sleep(0.5)
    servo.max()
    sleep(0.5)
