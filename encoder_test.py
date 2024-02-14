import pigpio
from time import sleep 
from gpiozero import RotaryEncoder

encoder1 = RotaryEncoder(22, 17)
encoder2 = RotaryEncoder(27, 16)

servo = 18
motor1 = 13
motor2 = 23
 
pwm = pigpio.pi() 
pwm.set_mode(servo, pigpio.OUTPUT)
pwm.set_mode(motor1, pigpio.OUTPUT)
pwm.set_mode(motor2, pigpio.OUTPUT)
 
pwm.set_PWM_frequency(servo, 50)
pwm.set_PWM_range(servo, 20000) # 1,000,000 / 50 = 20,000us for 100% duty cycle

pwm.set_PWM_frequency(motor1, 50)
pwm.set_PWM_frequency(motor2, 50)
pwm.set_PWM_range(motor1, 20000) # 1,000,000 / 50 = 20,000us for 100% duty cycle
pwm.set_PWM_range(motor2, 20000) # 1,000,000 / 50 = 20,000us for 100% duty cycle

pwm.hardware_PWM(servo, 50, 2000)
pwm.hardware_PWM(motor1, 50, 2000)
pwm.hardware_PWM(motor2, 50, 2000)
sleep(1)

while(1):
    pwm.set_PWM_dutycycle(motor1, 50)
    pwm.set_PWM_dutycycle(motor2, 50)
    sleep(1)

    if(encoder1.value > 0.5 or encoder2.value > 0.5 or encoder1.value < -0.5 or encoder2.value < -0.5):
        break

def fk_mqp(self):
    