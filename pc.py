import socket
import time
import tkinter as tk
import threading
import struct
import cv2
import numpy as np

cmd_port = 12345
ctrl_port = 54321
MCU_host = "192.168.12.122"
CD2_host = "192.168.12.44"
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

def send_custom_command():
    custom_command = custom_command_entry.get()
    CLIENT_send_immediate_command(MCU_host, custom_command)

def send_custom_command2():
    custom_command2 = custom_command_entry2.get()
    CLIENT_send_immediate_command(CD2_host, custom_command2)

def send_custom_command3():
    custom_command3 = custom_command_entry3.get()
    CLIENT_send_immediate_command(CD4_host, custom_command3)

def ch_dr(dr):
    global d
    d = dr

def servo(bool):
    if bool == True:
        CLIENT_send_immediate_command(MCU_host,"set_servo(0)")
    else:
        CLIENT_send_immediate_command(MCU_host,"set_servo(80)")

def CLIENT_CTRL(remote_host, cmd):
    global ctrl_port
    global d
    client_socket = socket.socket()
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        print("Send Control of drone to", cmd)
        client_socket.connect((remote_host, ctrl_port))
        client_socket.send(cmd.encode())
        
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


# Create the main GUI window
root = tk.Tk()
root.title("Drone Control")

# Create labels and buttons
label = tk.Label(root, text="Control the Drone")
label.grid(row=0, column=1, pady=10)

custom_command_label = tk.Label(root, text="Custom Command for MCU:")
custom_command_label.grid(row=1, column=0, padx=10, pady=5)

custom_command_entry = tk.Entry(root)
custom_command_entry.grid(row=1, column=1, padx=10, pady=5)

send_button = tk.Button(root, text="Send Custom Command to MCU", command=send_custom_command)
send_button.grid(row=1, column=2, padx=10, pady=5)

custom_command_label2 = tk.Label(root, text="Custom Command for CD2:")
custom_command_label2.grid(row=2, column=0, padx=10, pady=5)

custom_command_entry2 = tk.Entry(root)
custom_command_entry2.grid(row=2, column=1, padx=10, pady=5)

send_button2 = tk.Button(root, text="Send Custom Command to CD2", command=send_custom_command2)
send_button2.grid(row=2, column=2, padx=10, pady=5)

custom_command_label3 = tk.Label(root, text="Custom Command for CD4:")
custom_command_label3.grid(row=3, column=0, padx=10, pady=5)

custom_command_entry3 = tk.Entry(root)
custom_command_entry3.grid(row=3, column=1, padx=10, pady=5)

send_button3 = tk.Button(root, text="Send Custom Command to CD4", command=send_custom_command3)
send_button3.grid(row=3, column=2, padx=10, pady=5)

control_label = tk.Label(root, text="Control the Drone:")
control_label.grid(row=4, column=1, pady=10)

mcu_button = tk.Button(root, text="MCU", command=lambda: ch_dr('MCU'))
mcu_button.grid(row=5, column=0, padx=10, pady=5)

cd1_button = tk.Button(root, text="CD1", command=lambda: ch_dr('CD1'))
cd1_button.grid(row=6, column=0, padx=10, pady=5)

cd2_button = tk.Button(root, text="CD2", command=lambda: ch_dr('CD2'))
cd2_button.grid(row=5, column=1, padx=10, pady=5)

cd3_button = tk.Button(root, text="CD3", command=lambda: ch_dr('CD3'))
cd3_button.grid(row=6, column=1, padx=10, pady=5)

cd4_button = tk.Button(root, text="CD4", command=lambda: ch_dr('CD4'))
cd4_button.grid(row=5, column=2, padx=10, pady=5)

cd5_button = tk.Button(root, text="CD5", command=lambda: ch_dr('CD5'))
cd5_button.grid(row=6, column=2, padx=10, pady=5)

servo_on_button = tk.Button(root, text = "Camera ON", command=lambda: camera(True))
servo_on_button.grid(row=5, column=3, padx=10, pady=5)

servo_off_button = tk.Button(root, text = "Camera OFF", command=lambda: camera(False))
servo_off_button.grid(row=6, column=3, padx=10, pady=5)
def ctrl():
    root.unbind('m')
    root.unbind('l')
    root.unbind('t')
    root.unbind('w')
    root.unbind('a')
    root.unbind('d')
    root.unbind('u')
    root.unbind('j')
    root.unbind('s')

def ctrlON():
    root.bind('m', lambda event: send_command('m'))
    root.bind('l', lambda event: send_command('l'))
    root.bind('t', lambda event: send_command('t'))
    root.bind('w', lambda event: send_ctrl('w'))
    root.bind('a', lambda event: send_ctrl('a'))
    root.bind('d', lambda event: send_ctrl('d'))
    root.bind('u', lambda event: send_ctrl('u'))
    root.bind('j', lambda event: send_ctrl('j'))
    root.bind('s', lambda event: send_ctrl('s'))

ctrl_off_button = tk.Button(root, text = "CTRL OFF", command=lambda: ctrl())
ctrl_off_button.grid(row=7, column=0, padx=10, pady=5)

ctrl_on_button = tk.Button(root, text = "CTRL ON", command=lambda: ctrlON())
ctrl_on_button.grid(row=7, column=1, padx=10, pady=5)



def camera(c):
    global cmd
    cmd = c
    threading.Thread(target=(camera_init)).start()

cmd = False
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
                if cmd == False:
                    break
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

root.mainloop()