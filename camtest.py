import zmq
import io
import threading
import serial
import RPi.GPIO as GPIO
import time
import picamera

def send_video_footage():
    context2 = zmq.Context()
    video_socket = context2.socket(zmq.REP)
    video_socket.bind("tcp://*:5555")

    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)
        camera.framerate = 24
        time.sleep(2)  # Let the camera warm up
        stream = io.BytesIO()
        for _ in camera.capture_continuous(stream, format='jpeg', use_video_port=True):
            try:
                video_socket.send(stream.getvalue())
                video_socket.recv_string()
            except zmq.error.ZMQError as e:
                print("Error sending video footage:", e)
                break
            stream.seek(0)
            stream.truncate()

threading.Thread(target=send_video_footage).start()

GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)  # EN1
GPIO.setup(24, GPIO.OUT)  # IN1
GPIO.setup(25, GPIO.OUT)  # IN2
GPIO.setup(8, GPIO.OUT)   # IN3
GPIO.setup(7, GPIO.OUT)   # IN4
GPIO.setup(12, GPIO.OUT)  # EN2


GPIO.output(23, GPIO.LOW)
GPIO.output(12, GPIO.LOW)

def perform():
    context1 = zmq.Context()
    action_socket = context1.socket(zmq.REP)
    action_socket.bind("tcp://*:5566")
    while True:
        fingers = action_socket.recv_string()  # Receive the number of fingers as an integer
        print(f"Fingers: {fingers}")

        # Motor control logic based on the number of fingers
        if fingers == '1' or fingers == 1:
            # Stop motors
            GPIO.output(23, GPIO.LOW)
            GPIO.output(12, GPIO.LOW)

        elif fingers == '3' or fingers == 3:
            # Run motors in one direction
            GPIO.output(23, GPIO.HIGH)  # Enable motors
            GPIO.output(24, GPIO.HIGH)  # Set direction
            GPIO.output(25, GPIO.LOW)   # Set direction
            GPIO.output(12, GPIO.HIGH)  # Enable motors
            GPIO.output(8, GPIO.HIGH)   # Set direction
            GPIO.output(7, GPIO.LOW)    # Set direction

        elif fingers == '5' or fingers == 5:
            # Run motors in one direction
            GPIO.output(23, GPIO.HIGH)  # Enable motors
            GPIO.output(24, GPIO.LOW)  # Set direction
            GPIO.output(25, GPIO.HIGH)   # Set direction
            GPIO.output(12, GPIO.HIGH)  # Enable motors
            GPIO.output(8, GPIO.LOW)   # Set direction
            GPIO.output(7, GPIO.HIGH)    # Set direction

        elif fingers == '2l':
            GPIO.output(23, GPIO.HIGH)  # Enable motors
            GPIO.output(24, GPIO.LOW)  # Set direction
            GPIO.output(25, GPIO.HIGH)   # Set direction
            GPIO.output(12, GPIO.HIGH)  # Enable motors
            GPIO.output(8, GPIO.HIGH)   # Set direction
            GPIO.output(7, GPIO.LOW)    # Set direction

        elif fingers == '2r':
            GPIO.output(23, GPIO.HIGH)  # Enable motors
            GPIO.output(24, GPIO.HIGH)  # Set direction
            GPIO.output(25, GPIO.LOW)   # Set direction
            GPIO.output(12, GPIO.HIGH)  # Enable motors
            GPIO.output(8, GPIO.LOW)   # Set direction
            GPIO.output(7, GPIO.HIGH)    # Set direction

        elif fingers == '4':
            GPIO.output(23, GPIO.HIGH)  # Enable motors
            GPIO.output(24, GPIO.LOW)  # Set direction
            GPIO.output(25, GPIO.HIGH)   # Set direction
            GPIO.output(12, GPIO.HIGH)  # Enable motors
            GPIO.output(8, GPIO.LOW)   # Set direction
            GPIO.output(7, GPIO.HIGH)    # Set direction
            time.sleep(0.3)
            GPIO.output(24, GPIO.HIGH)  # Set direction
            GPIO.output(25, GPIO.LOW)   # Set direction
            GPIO.output(8, GPIO.LOW)   # Set direction
            GPIO.output(7, GPIO.HIGH)    # Set direction
            time.sleep(0.3)
            GPIO.output(24, GPIO.LOW)  # Set direction
            GPIO.output(25, GPIO.HIGH)   # Set direction
            GPIO.output(8, GPIO.LOW)   # Set direction
            GPIO.output(7, GPIO.HIGH)    # Set direction
            time.sleep(0.3)
            GPIO.output(24, GPIO.LOW)  # Set direction
            GPIO.output(25, GPIO.HIGH)   # Set direction
            GPIO.output(8, GPIO.HIGH)   # Set direction
            GPIO.output(7, GPIO.LOW)    # Set direction
            time.sleep(0.3)
            GPIO.output(24, GPIO.LOW)  # Set direction
            GPIO.output(25, GPIO.HIGH)   # Set direction
            GPIO.output(8, GPIO.LOW)   # Set direction
            GPIO.output(7, GPIO.HIGH)    # Set direction

        else:
            GPIO.output(23, GPIO.LOW)
            GPIO.output(12, GPIO.LOW)

        action_socket.send_string('OK')


perform()


GPIO.cleanup()  # Clean up GPIO pins
