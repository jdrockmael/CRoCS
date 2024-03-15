from drivers.motor_dr import motor
from time import sleep

motor.drive(0.5, -0.5)
sleep(1)
motor.stop()