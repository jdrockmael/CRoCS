#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32MultiArray, Bool
from time import sleep

speed_pub = rospy.Publisher("robot_twist", Float32MultiArray, queue_size=1)
motor_stop = rospy.Publisher("kill_motors", Bool, queue_size=1)

cam_readings = None # heading and distance
lock_on = True

def update_reading(cam : Float32MultiArray):
    global cam_readings
    data = cam.data

    if len(data) != 0:
        cam_readings = (data[2], data[1])
    else:
        cam_readings = None

def control_loop():
    tolerance = 0.1
    vel = 1
    head = None
    sign_of_head = None

    while(not rospy.is_shutdown() and lock_on):
        if cam_readings != None:
            head = cam_readings[0]
            sign_of_head = head / abs(head) if head != 0 else (head + 1) / (abs(head) + 1)

            rospy.logerr(cam_readings)

            if abs(head) > tolerance:
                speed_pub.publish(Float32MultiArray(data=[0.0, (sign_of_head * vel)]))
                sleep(0.01)
                motor_stop.publish(Bool(data=True))
                sleep(0.1)
            else:
                motor_stop.publish(Bool(data=True))
    
    speed_pub.publish(Float32MultiArray(data=[0.0, 0.0]))

if __name__ == '__main__':
    rospy.init_node('lock')
    rospy.Subscriber("range", Float32MultiArray, update_reading)
    while cam_readings == None:
        pass
    control_loop()
    rospy.spin()
    