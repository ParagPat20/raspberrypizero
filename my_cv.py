import io
import picamera
import socket
import struct
import time

# Create a socket server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', 8000))
server_socket.listen(0)

# Accept a single connection from the client
connection = server_socket.accept()[0].makefile('wb')

try:
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)  # Adjust resolution as needed
        camera.framerate = 30  # Adjust frame rate as needed

        # Start capturing and sending the video feed
        time.sleep(2)  # Give the camera some time to warm up
        stream = io.BytesIO()
        for _ in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
            stream.seek(0)
            image_data = stream.read()

            # Send the image size to the client
            connection.write(struct.pack('<L', len(image_data)))
            connection.flush()

            # Send the image data to the client
            connection.write(image_data)
            stream.seek(0)
            stream.truncate()
except Exception as e:
    print("Error : ", e)

finally:
    connection.close()
    server_socket.close()
