import socket
import time
import tkinter as tk
import keyboard
import threading

cmd_port = 12345
ctrl_port = 54321  # Set your control port

MCU_host = "192.168.149.101"
CD2_host = "192.168.149.102"
CD4_host = "192.168.149.103"

def CLIENT_send_immediate_command(remote_host, immediate_command_str):
    global cmd_port

    client_socket = socket.socket()
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        client_socket.connect((remote_host, cmd_port))
    except socket.error as error_msg:
        print('{} - Caught exception : {}'.format(time.ctime(), error_msg))
        print('{} - CLIENT_send_immediate_command({}) is not executed!'.format(time.ctime(), immediate_command_str))
        return

    client_socket.send(immediate_command_str.encode())  # Encode the command as bytes before sending

def send_command(command):
    if command == 'm':
        threading.Thread(target=send, args=('ARM(drone1)',)).start()
        threading.Thread(target=send, args=('ARM(drone2)',)).start()
    elif command == 'l':
        threading.Thread(target=send, args=('land_all()',)).start()
    elif command == 't':
        threading.Thread(target=send, args=('TAKEOFF(drone1)',)).start()
        threading.Thread(target=send, args=('TAKEOFF(drone2)',)).start()
    elif command == 'mcu':
        CLIENT_send_immediate_command(MCU_host, 'D(MCU)')
    elif command == 'cd1':
        CLIENT_send_immediate_command(MCU_host, 'D(CD1)')
    elif command == 'cd2':
        CLIENT_send_immediate_command(CD2_host, 'D(CD2)')
    elif command == 'cd3':
        CLIENT_send_immediate_command(CD2_host, 'D(CD3)')
    elif command == 'cd4':
        CLIENT_send_immediate_command(CD4_host, 'D(CD4)')
    elif command == 'cd5':
        CLIENT_send_immediate_command(CD4_host, 'D(CD5)')

def send(cmd):
    CLIENT_send_immediate_command(MCU_host, cmd)
    CLIENT_send_immediate_command(CD2_host, cmd)
    CLIENT_send_immediate_command(CD4_host, cmd)

def send_custom_command():
    custom_command = custom_command_entry.get()
    CLIENT_send_immediate_command(MCU_host, custom_command)

def CLIENT_CTRL(remote_host):
    global ctrl_port
    # Create a socket object
    client_socket = socket.socket()
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        client_socket.connect((remote_host, ctrl_port))
    except socket.error as error_msg:
        print('{} - Caught exception : {}'.format(time.ctime(), error_msg))
        print('{} - ClientSendCtrl({}, {}) is not executed!'.format(time.ctime(), remote_host))
        return
    client_socket.send(ctrl().encode())  # Encode the string as bytes before sending

def ctrl():
    Velocity = 0.5

    if keyboard.is_pressed('w'):
        x = str(Velocity)
    elif keyboard.is_pressed('s'):
        x = str(-Velocity)
    else:
        x = '0'

    if keyboard.is_pressed('a'):
        y = str(Velocity)
    elif keyboard.is_pressed('d'):
        y = str(-Velocity)
    else:
        y = '0'

    if keyboard.is_pressed('u'):
        z = str(-Velocity)
    elif keyboard.is_pressed('j'):
        z = str(Velocity)
    else:
        z = '0'

    ctrl_str = x + ',' + y + ',' + z

    return ctrl_str

# Create the main GUI window
root = tk.Tk()
root.title("Command Sender")

# Create labels and buttons
label = tk.Label(root, text="Press 'M' for ARM,\n 'L' for LAND,\n 'T' for TAKEOFF")
label.pack()

custom_command_label = tk.Label(root, text="Custom Command:")
custom_command_label.pack()

custom_command_entry = tk.Entry(root)
custom_command_entry.pack()

send_button = tk.Button(root, text="Send Custom Command", command=send_custom_command)
send_button.pack()

# Buttons to send commands as MCU, CD1, CD2, CD3, CD4, CD5
mcu_button = tk.Button(root, text="MCU", command=lambda: send_command('mcu'))
cd1_button = tk.Button(root, text="CD1", command=lambda: send_command('cd1'))
cd2_button = tk.Button(root, text="CD2", command=lambda: send_command('cd2'))
cd3_button = tk.Button(root, text="CD3", command=lambda: send_command('cd3'))
cd4_button = tk.Button(root, text="CD4", command=lambda: send_command('cd4'))
cd5_button = tk.Button(root, text="CD5", command=lambda: send_command('cd5'))

mcu_button.pack()
cd1_button.pack()
cd2_button.pack()
cd3_button.pack()
cd4_button.pack()
cd5_button.pack()

# Bind key events to functions
root.bind('m', lambda event: send_command('m'))
root.bind('l', lambda event: send_command('l'))
root.bind('t', lambda event: send_command('t'))

# Start the GUI main loop
root.mainloop()
