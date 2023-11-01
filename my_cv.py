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

    try:
        with picamera.PiCamera() as camera:
            camera.resolution = (640, 480)  # Adjust resolution as needed
            camera.framerate = 30  # Adjust frame rate as needed

            # Start capturing and sending the video feed
            time.sleep(2)  # Give the camera some time to warm up
            stream = io.BytesIO()

            while True:
                stream.seek(0)
                camera.capture(stream, 'jpeg', use_video_port=True)
                image_data = stream.getvalue()

                # Send the image size to the client
                connection, client_address = camera_socket.accept()
                connection_file = connection.makefile('wb')
                connection_file.write(struct.pack('<L', len(image_data)))
                connection_file.flush()

                # Send the image data to the client
                connection_file.write(image_data)
                connection_file.flush()

                # Close the client connection
                connection_file.close()
                connection.close()

                stream.seek(0)
                stream.truncate()

    except Exception as e:
        print("Error: ", e)

    finally:
        camera_socket.close()

camera_init()
