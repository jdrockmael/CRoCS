from gpiozero import PhaseEnableMotor, Servo
#motor forward ccw, backward cw
from time import sleep

servo = Servo(7)
motor1 = PhaseEnableMotor(26, 13)
motor2 = PhaseEnableMotor(24, 23)

servo.min()
sleep(0.5)
servo.mid()
sleep(0.5)
servo.max()
sleep(0.5)

for i in range(1,10):
    motor1.forward(i/10)
    sleep(2)
    motor2.backward(i/10)
    sleep(2)
    i+=1