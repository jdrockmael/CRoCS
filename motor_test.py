from gpiozero import PhaseEnableMotor, Servo
#motor forward ccw, backward cw
from time import sleep
import rospy
from motor_test import ROSMotor

if __name__ == "__main__":
    rospy.init_node("motor_driver")
    motor = ROSMotor()
    rospy.on_shutdown(motor.stop)
    rospy.loginfo("Motor driver is now started, ready to get commands.")
    rospy.spin()

class ROSMotor:
    def __init__(self):
        servo = Servo(25)
        motor1 = PhaseEnableMotor(26, 13)
        motor2 = PhaseEnableMotor(24, 23)

        servo.min()
        sleep(1)

    def forward(self, time):
        self.motor1.forward(0.5)
        self.motor2.backward(0.5)
        sleep(time)

    def backward(self, time):
        self.motor1.backward(0.5)
        self.motor2.forward(0.5)
        sleep(time)
    
    def close_gripper(self):
        self.servo.max()
        sleep(1)

    def open_gripper(self):
        self.servo.min()
        sleep(1)

    def stop(self):
        self.motor1.stop()
        self.motor2.stop()
