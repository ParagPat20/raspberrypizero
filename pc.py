import socket
import time
import tkinter as tk
import keyboard
import threading

cmd_port = 12345
ctrl_port = 54321  # Set your control port

MCU_host = "192.168.149.101"

d='MCU'

def CLIENT_send_immediate_command(remote_host, immediate_command_str):
    global cmd_port

    client_socket = socket.socket()
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        print("Sending Command",immediate_command_str)
        client_socket.connect((remote_host, cmd_port))
        client_socket.send(immediate_command_str.encode())  # Encode the command as bytes before sending
        
    except socket.error as error_msg:
        print('{} - Caught exception : {}'.format(time.ctime(), error_msg))
        print('{} - CLIENT_send_immediate_command({}) is not executed!'.format(time.ctime(), immediate_command_str))
        return
    
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

def ch_dr(dr):
    global d
    d = dr

def CLIENT_CTRL(remote_host):
    global ctrl_port
    global d
    # Create a socket object
    client_socket = socket.socket()
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        print("Send Control of drone to",ctrl())
        client_socket.connect((remote_host, ctrl_port))
        
        client_socket.send(ctrl().encode())  # Encode the string as bytes before sending
        
    except socket.error as error_msg:
        print('{} - Caught exception : {}'.format(time.ctime(), error_msg))
        print('{} - ClientSendCtrl({}, {}) is not executed!'.format(time.ctime(), remote_host))
        return
    
def ctrl():
    Velocity = 0.5
    global d
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

    ctrl_str = d + ',' + x + ',' + y + ',' + z

    return ctrl_str

def send_ctrl():
    while True:
        if keyboard.is_pressed('w') or keyboard.is_pressed('s') or keyboard.is_pressed('a') or keyboard.is_pressed('d') or keyboard.is_pressed('u') or keyboard.is_pressed('j'):
            CLIENT_CTRL(MCU_host)

threading.Thread(target=send_ctrl).start()

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
mcu_button = tk.Button(root, text="MCU", command=lambda: ch_dr('MCU'))
cd1_button = tk.Button(root, text="CD1", command=lambda: ch_dr('CD1'))
cd2_button = tk.Button(root, text="CD2", command=lambda: ch_dr('CD2'))
cd3_button = tk.Button(root, text="CD3", command=lambda: ch_dr('CD3'))
cd4_button = tk.Button(root, text="CD4", command=lambda: ch_dr('CD4'))
cd5_button = tk.Button(root, text="CD5", command=lambda: ch_dr('CD5'))

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
