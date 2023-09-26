from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import socket

C = {}
P = {
    1:{'Batt': 0, 'Groundspeed': 0, 'ARM': 0, 'GPS': 0, 'Altitude': 0, 'MODE': None, 'VelocityX': 0, 'VelocityY': 0, 'VelocityZ': 0},
    2:{'Batt': 0, 'Groundspeed': 0, 'ARM': 0, 'GPS': 0, 'Altitude': 0, 'MODE': None, 'VelocityX': 0, 'VelocityY': 0, 'VelocityZ': 0}
}


def control(controling_drone):
    if C['Arming'] == 1:
        controling_drone.arm()

def Client_Start(server_ip, server_port):
    global C,P
    
    # Create a socket object and connect to the server
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, server_port))

    c_str = client_socket.recv(1024).decode()
    C = eval(c_str) 
    print(C) # Convert the received string back to a dictionary
    print("Connected to the server")
    drone1_init = False
    drone2_init = False
    while True:
        # Receive C dictionary values from the server
        c_str = client_socket.recv(1024).decode()
        C = eval(c_str)  # Convert the received string back to a dictionary
        print(C)
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
