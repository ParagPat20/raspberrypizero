import io
import picamera
import socket
import struct
import time
local_host = '192.168.12.122'

def camera_init():
    camera_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    camera_socket.bind((local_host, 8888))
    camera_socket.listen(1)

    while True:
        client_connection, client_address = camera_socket.accept()
        connection = client_connection.makefile('wb')

        try:
            with picamera.PiCamera() as camera:
                camera.resolution = (640, 480)  # Adjust resolution as needed
                camera.framerate = 30  # Adjust frame rate as needed

                # Start capturing and sending the video feed
                time.sleep(2)  # Give the camera some time to warm up
                stream = io.BytesIO()

                while True:
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
            client_connection.close()
            camera_socket.close()

camera_init()