import io
import picamera
import socket
import struct
import time
import threading

def camera_stream_server(host, port):
    def handle_client(client_socket):
        connection = client_socket.makefile('wb')

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
            print("Error: ", e)

        finally:
            connection.close()
            client_socket.close()

    # Create a socket server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(0)

    print("Server is listening on {}:{}".format(host, port))

    while True:
        client_socket, _ = server_socket.accept()
        client_thread = threading.Thread(target=handle_client, args=(client_socket,))
        client_thread.start()

# Usage: Call the function to start the camera stream server
if __name__ == "__main__":
    camera_stream_server('192.168.12.122', 8000)
