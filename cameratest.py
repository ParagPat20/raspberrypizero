import zmq
import picamera
import io
import threading

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.bind("tcp://*:5555")
fingers = 0
action_socket = context.socket(zmq.REQ)
action_socket.bind("tcp://*:5556")

def perform():
    while True:
        fingers=action_socket.recv_string()
        print(fingers)
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


