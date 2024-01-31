from gpiozero import PhaseEnableMotor, Servo
#motor forward ccw, backward cw
from time import sleep

servo = Servo(25)
motor1 = PhaseEnableMotor(26, 13)
motor2 = PhaseEnableMotor(24, 23)

servo.min()
sleep(1)

servo.max()
motor1.forward(0.5)
motor2.backward(0.5)
sleep(2)
