#GPIO23 = pin 16 - Servo

# Set up libraries and overall settings
import RPi.GPIO as GPIO  # Imports the standard Raspberry Pi GPIO library
from time import sleep   # Imports sleep (aka wait or pause) into the program
GPIO.setmode(GPIO.BOARD) # Sets the pin numbering system to use the physical layout

# Set up pin 11 for PWM
GPIO.setup(16,GPIO.OUT)  # Sets up pin 16 to an output (instead of an input)

servo = GPIO.PWM(16, 50)     # Sets up pin 16 as a PWM pin

servo.start(0)               # Starts running PWM on the pin and sets it to 0


# Move the servo back and forth
servo.ChangeDutyCycle(3)     # Changes the pulse width to 3 (so moves the servo)
sleep(1)                 # Wait 1 second
servo.ChangeDutyCycle(12)
sleep(1)

# Clean up everything
servo.stop()                 # At the end of the program, stop the PWM
GPIO.cleanup()           # Resets the GPIO pins back to defaults