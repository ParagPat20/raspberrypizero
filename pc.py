import socket
import time
import threading
import struct
import cv2
import numpy as np

cmd_port = 12345
ctrl_port = 54321
MCU_host = "192.168.12.122"
CD2_host = "192.168.149.43"
CD4_host = "192.168.149.103"

d = 'MCU'

def CLIENT_send_immediate_command(remote_host, immediate_command_str):
    global cmd_port

    client_socket = socket.socket()
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        print("Sending Command", immediate_command_str)
        client_socket.connect((remote_host, cmd_port))
        client_socket.send(immediate_command_str.encode())
        
    except socket.error as error_msg:
        print('{} - Caught exception: {}'.format(time.ctime(), error_msg))
        print('{} - CLIENT_send_immediate_command({}) is not executed!'.format(time.ctime(), immediate_command_str))
        return
    
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(1)
    

def send_command(command):
    if command == 'm':
        threading.Thread(target=send, args=('ARMALL()',)).start()
    elif command == 'l':
        threading.Thread(target=send, args=('LANDALL()',)).start()
    elif command == 't':
        threading.Thread(target=send, args=('TAKEOFFALL()',)).start()

def send(cmd):
    CLIENT_send_immediate_command(MCU_host, cmd)

def send(cmd):
    CLIENT_send_immediate_command(CD2_host, cmd)

def send(cmd):
    CLIENT_send_immediate_command(CD4_host, cmd)


def ch_dr(dr):
    global d
    d = dr

def CLIENT_CTRL(remote_host, cmd):
    global ctrl_port
    global d

    client_socket = socket.socket()
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        print("Send Control of drone to", cmd)
        client_socket.connect((remote_host, ctrl_port))
        client_socket.send(cmd)
        
    except socket.error as error_msg:
        print('{} - Caught exception: {}'.format(time.ctime(), error_msg))
        return
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(1)

def send_ctrl(cmd):
    Velocity = 0.5
    global d

    x = '0'
    y = '0'
    z = '0'

    if cmd == 'w':
        x = str(Velocity)
    elif cmd == 's':
        x = str(-Velocity)

    if cmd == 'a':
        y = str(Velocity)
    elif cmd == 'd':
        y = str(-Velocity)

    if cmd == 'u':
        z = str(-Velocity)
    elif cmd == 'j':
        z = str(Velocity)

    cmd = d + ',' + x + ',' + y + ',' + z
    CLIENT_CTRL(MCU_host, cmd)


def camera_init():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((MCU_host, 8000))  # Replace with your Raspberry Pi's IP address

    connection = client_socket.makefile('rb')

    try:
        while True:
            # Read the image size from the server
            image_len_data = connection.read(struct.calcsize('<L'))
            if not image_len_data:
                break  # Break the loop if no image data is received
            image_len = struct.unpack('<L', image_len_data)[0]

            # Read the image data from the server
            image_data = connection.read(image_len)

            # Convert the image data to a NumPy array
            image_array = np.frombuffer(image_data, dtype=np.uint8)

            # Decode the image as a color image (you may need to adjust the format)
            if image_array.size > 0:
                image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                cv2.imshow('Video Stream', image)

            # Press 'q' to quit the video stream
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(e)

camera_feed_thread = threading.Thread(target=camera_init)
camera_feed_thread.daemon = True
camera_feed_thread.start()

