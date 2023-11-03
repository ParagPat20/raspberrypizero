import time
import RPi.GPIO as GPIO
import threading
servo = 13
# Set up the GPIO pin to control the servo motor
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo, GPIO.OUT)

# Create a PWM object on the GPIO pin
pwm = GPIO.PWM(servo, 50)
pwm.start(0)
angle = 0
def set_angle():
    while True:
        global angle
        degree = int(input("Enter Degree: "))
        angle = degree

def set_servo():
    try:
        while True:
            duty = (angle/18)+2.5
            # Rotate the servo motor to 0 degrees
            pwm.ChangeDutyCycle(duty)

    except KeyboardInterrupt:
        pass

    finally:
        # Stop the PWM object and clean up the GPIO
        pwm.stop()
        GPIO.cleanup()

threading.Thread(target=set_servo).start()

set_angle()
