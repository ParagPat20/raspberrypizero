import zmq
import picamera
import io
import threading
import serial

# Setup serial connection
ser = serial.Serial('/dev/serial0', 9600)  # Adjust the port and baud rate as needed

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.bind("tcp://*:5555")
fingers = 0

def perform():
    context1 = zmq.Context()
    action_socket = context1.socket(zmq.REP)
    action_socket.bind("tcp://*:5566")
    while True:
        fingers = action_socket.recv_string()
        print(fingers)
        action_socket.send_string('OK')

        # Send the number of fingers to the Arduino Nano over serial
        ser.write(fingers.encode())

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
