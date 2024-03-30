import socket
import picamera
import io

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('192.168.67.229', 8000))
server_socket.listen(0)

connection = server_socket.accept()[0].makefile('wb')

try:
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)
        camera.framerate = 30
        stream = io.BytesIO()
        
        for _ in camera.capture_continuous(stream, format='jpeg', use_video_port=True):
            connection.write(stream.getvalue())
            stream.seek(0)
            stream.truncate()
finally:
    connection.close()
    server_socket.close()
