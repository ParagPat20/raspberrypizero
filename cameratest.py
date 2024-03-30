import zmq
import picamera
import io
import time

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")

with picamera.PiCamera() as camera:
    camera.resolution = (640, 480)
    camera.framerate = 10  # Lowering the framerate to reduce data size
    stream = io.BytesIO()
    
    for _ in camera.capture_continuous(stream, format='jpeg', use_video_port=True):
        socket.send(stream.getvalue(), flags=zmq.SNDMORE)  # Sending frame
        socket.send_string('latest', flags=0)  # Sending flag
        
        time.sleep(0.1)  # Wait for a short time to reduce the rate of frames sent
        stream.seek(0)
        stream.truncate()
