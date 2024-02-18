import pigpio
from time import sleep 
from gpiozero import RotaryEncoder, Device
from gpiozero.pins.pigpio import PiGPIOFactory

Device.pin_factory = PiGPIOFactory()

encoder1 = RotaryEncoder(5, 6)
encoder2 = RotaryEncoder(20, 21)

servo = 18
motor1 = 23
motor2 = 13
 
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
pwm.hardware_PWM(motor1, 50, 10000)
pwm.hardware_PWM(motor2, 50, 10000)
sleep(1)

pwm.set_servo_pulsewidth(servo, 500)
sleep(1)
pwm.set_servo_pulsewidth(servo, 2500)
sleep(1)

pwm.set_PWM_dutycycle(motor1, 50)
pwm.set_PWM_dutycycle(motor2, 50)
sleep(2)
print(encoder1.value)
print(encoder2.value)
