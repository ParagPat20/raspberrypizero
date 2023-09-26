from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import socket

C = {'Drone':0, 'vx': 0, 'vy': 0, 'vz': 0, 'Arming': 0, 'Mode': 'GUIDED', 'Takeoff': 0, 'mstart': False}
P = {
    1:{'Batt': 0, 'Groundspeed': 0, 'ARM': 0, 'GPS': 0, 'Altitude': 0, 'MODE': None, 'VelocityX': 0, 'VelocityY': 0, 'VelocityZ': 0},
    2:{'Batt': 0, 'Groundspeed': 0, 'ARM': 0, 'GPS': 0, 'Altitude': 0, 'MODE': None, 'VelocityX': 0, 'VelocityY': 0, 'VelocityZ': 0}
}


def control(controling_drone):
    if C['Arming'] == 1:
        controling_drone.arm()

    if C['Takeoff'] == 1:
        controling_drone.arm()
        controling_drone.takeoff()
    
    if P[controling_drone]['MODE'] != 'VehicleMode:' + C['Mode']:
        controling_drone.vehicle.mode = VehicleMode(C['Mode'])

    controling_drone.send_ned_velocity(C['vx'], C['vy'], C['vz'], 1)

def Client_Start(server_ip, server_port):
    global C,P
    c_str = client_socket.recv(2048).decode()
    C = eval(c_str)  # Convert the received string back to a dictionary
    # Create a socket object and connect to the server
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, server_port))
    print("Connected to the server")
    if C['Drone'] == 1:
            print("D1 initializing")
            # D1 = connect('tcp:127.0.0.1:5762')
            D1 = connect('/dev/serial0', baud= 115200)
            print("D1 Initialized")

    c_str = client_socket.recv(2048).decode()
    C = eval(c_str)  # Convert the received string back to a dictionary

    if C['Drone'] == 2:
        print("D2 initializing")
        # D2 = connect('tcp:127.0.0.1:5772')
        D2 = connect('0.0.0.0:14550')
        print("D2 Initialized")
    if C['mstart'] == 1:   
        while True:
            # Receive C dictionary values from the server
            c_str = client_socket.recv(2048).decode()
            C = eval(c_str)  # Convert the received string back to a dictionary
            
            if C['Drone'] == 1:
                control(D1)

            if C['Drone'] == 2:
                control(D2)

            time.sleep(0.5)  # Adjust the sleep interval as needed

Client_Start('192.168.14.101',12345)
