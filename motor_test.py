#pin 35 - Motor FWD
#pin 37 - Motor REV
#2.5 = min, 12.5 = max

# Set up libraries and overall settings
import RPi.GPIO as GPIO  # Imports the standard Raspberry Pi GPIO library
from time import sleep   # Imports sleep (aka wait or pause) into the program
GPIO.setmode(GPIO.BOARD) # Sets the pin numbering system to use the physical layout

# Set up pin 16 for PWM
GPIO.setup(33,GPIO.OUT)  # Sets up pin 16 to an output (instead of an input)
GPIO.setup(37,GPIO.OUT)  # Sets up pin 16 to an output (instead of an input)

motor_fwd = GPIO.PWM(33, 50)     # Sets up pin 16 as a PWM pin
motor_rev = GPIO.PWM(37, 50)     # Sets up pin 16 as a PWM pin

motor_fwd.start(0)               # Starts running PWM on the pin and sets it to 0
motor_rev.start(0)               # Starts running PWM on the pin and sets it to 0


# Move the motor back and forth
motor_fwd.ChangeDutyCycle(2.5)     # Changes the pulse width to 3 (so moves the motor)
sleep(2)                 # Wait 1 second
motor_fwd.ChangeDutyCycle(12.5)
sleep(2)
motor_rev.ChangeDutyCycle(12.5)     # Changes the pulse width to 3 (so moves the motor)
sleep(2)                 # Wait 1 second
motor_rev.ChangeDutyCycle(2.5)
sleep(2)

# Clean up everything
motor_fwd.stop()                 # At the end of the program, stop the PWM
motor_rev.stop()                 # At the end of the program, stop the PWM
GPIO.cleanup()           # Resets the GPIO pins back to defaults