from gpiozero import Motor
import time
motor1 = Motor(4, 14)

motor1.forward()
while 1:
    time.sleep(5)
    motor1.reverse()