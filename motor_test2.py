from gpiozero import PhaseEnableMotor
#forward ccw, backward cw
from time import sleep

motor1 = PhaseEnableMotor(26, 13)
motor2 = PhaseEnableMotor(24, 23)

for i in range(1,10):
    motor1.forward(i/10)
    sleep(2)
    motor2.backward(i/10)
    sleep(2)
    i+=1
