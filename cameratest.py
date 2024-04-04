import zmq
import picamera
import io
import threading
import serial
import RPi.GPIO as GPIO


GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)  # EN1
GPIO.setup(24, GPIO.OUT)  # IN1
GPIO.setup(25, GPIO.OUT)  # IN2
GPIO.setup(8, GPIO.OUT)   # IN3
GPIO.setup(7, GPIO.OUT)   # IN4
GPIO.setup(12, GPIO.OUT)  # EN2

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.bind("tcp://*:5555")

def perform():
    context1 = zmq.Context()
    action_socket = context1.socket(zmq.REP)
    action_socket.bind("tcp://*:5566")
    while True:
        fingers = int(action_socket.recv_string())  # Receive the number of fingers as an integer
        print(f"Fingers: {fingers}")

        # Motor control logic based on the number of fingers
        if fingers == 0:
            # Stop motors
            GPIO.output(23, GPIO.LOW)
            GPIO.output(12, GPIO.LOW)
        elif fingers > 0:
            # Run motors in one direction
            GPIO.output(23, GPIO.HIGH)  # Enable motors
            GPIO.output(24, GPIO.HIGH)  # Set direction
            GPIO.output(25, GPIO.LOW)   # Set direction
            GPIO.output(12, GPIO.HIGH)  # Enable motors
            GPIO.output(8, GPIO.HIGH)   # Set direction
            GPIO.output(7, GPIO.LOW)    # Set direction
        else:
            # Run motors in the other direction
            GPIO.output(23, GPIO.HIGH)  # Enable motors
            GPIO.output(24, GPIO.LOW)   # Set direction
            GPIO.output(25, GPIO.HIGH)  # Set direction
            GPIO.output(12, GPIO.HIGH)  # Enable motors
            GPIO.output(8, GPIO.LOW)    # Set direction
            GPIO.output(7, GPIO.HIGH)   # Set direction

        action_socket.send_string('OK')

with picamera.PiCamera() as camera:
    camera.resolution = (640, 480)
    camera.framerate = 30
    stream = io.BytesIO()
    threading.Thread(target=perform).start()
    for _ in camera.capture_continuous(stream, format='jpeg', use_video_port=True):
        socket.send(stream.getvalue())
        stream.seek(0)
        stream.truncate()
        socket.recv_string()

GPIO.cleanup()  # Clean up GPIO pins
