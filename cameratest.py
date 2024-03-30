import zmq
import picamera
import io

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.bind("tcp://*:5555")

with picamera.PiCamera() as camera:
    camera.resolution = (640, 480)
    camera.framerate = 30
    stream = io.BytesIO()
    for _ in camera.capture_continuous(stream, format='jpeg', use_video_port=True):
        socket.send(stream.getvalue())
        stream.seek(0)
        stream.truncate()
        socket.recv_string()
