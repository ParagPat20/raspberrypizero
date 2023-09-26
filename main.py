from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import threading
import socket

C = {}


def control(controling_drone):
    if C['Arming'] == 1:
        controling_drone.arm()

def send(client_socket):
    # Receive C dictionary values from the server
    global C
    c_str = client_socket.recv(1024).decode()
    C = eval(c_str)  # Convert the received string back to a dictionary
    print(C)

def Client_Start(server_ip, server_port):
    global C
    
    # Create a socket object and connect to the server
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, server_port))

    c_str = client_socket.recv(1024).decode()
    C = eval(c_str) 
    print(C) # Convert the received string back to a dictionary
    print("Connected to the server")
    drone1_init = False
    drone2_init = False
    thread = threading.Thread(target=send,args=(client_socket,))
    thread.start()
    while True:
        if C['Drone'] == 1:
            if drone1_init == False:
                print("D1 initializing")
                # D1 = connect('tcp:127.0.0.1:5762')
                D1 = connect('/dev/serial0', baud= 115200)
                drone1_init = True
                print("D1 Initialized")
            control(D1)
            

        if C['Drone'] == 2:
            if drone2_init == False:
                print("D2 initializing")
                # D2 = connect('tcp:127.0.0.1:5772')
                D2 = connect('0.0.0.0:14550')
                drone2_init = True
                print("D2 Initialized")
            control(D2)
        
        time.sleep(0.5)
            


Client_Start('192.168.14.101',12345)
