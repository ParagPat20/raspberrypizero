# Variables for drone
P = {
    0: {'Batt': 0, 'Groundspeed': 0, 'ARM': 0, 'GPS': 0, 'Altitude': 0, 'MODE': None, 'VelocityX': 0, 'VelocityY': 0, 'VelocityZ': 0}
}

C = {
    0: {'vx': 0, 'vy': 0, 'vz': 0, 'Arming': 0, 'Mode': 'GUIDED', 'Takeoff': 0},
    'drone': None
}

from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import socket

class Drone:
    def __init__(self, connection_string, baudrate=None):
        self.vehicle = connect(connection_string, baud=baudrate)

    # ... (rest of the methods remain the same)

def Client_Start(server_ip, server_port):
    # Create a socket object and connect to the server
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, server_port))
    print("Connected to the server")
    my_drone = None
    drone1_init = False
    while True:
        p_str1 = str(P[0])
        client_socket.send(p_str1.encode())

        # Receive C dictionary values from the server
        c_str = client_socket.recv(2048).decode()
        control_params = eval(c_str)  # Convert the received string back to a dictionary
        c_drone = client_socket.recv(1024).decode()
        droneid = eval(c_drone) # Convert the received string back

        if droneid == 0:
            if drone1_init == False:
                my_drone = Drone('/dev/serial0', baudrate=115200)
                print("Main Drone initialized")
                drone1_init = True
            Control(my_drone, control_params['0'])  # Pass the control parameters for the drone

        time.sleep(0.5)  # Adjust the sleep interval as needed

def Control(drone, control_params):
    if control_params['Mode'] == 'GUIDED' and P['ARM'] == 0 and control_params['Arming'] == 1:
        drone.arm(mode='GUIDED')

    if control_params['Takeoff'] == 1:
        drone.arm(mode='GUIDED')
        drone.takeoff()
        print("Here")

    if P['MODE'] != 'VehicleMode:' + control_params['Mode']:
        drone.vehicle.mode = VehicleMode(control_params['Mode'])

    drone.send_ned_velocity(control_params['vx'], control_params['vy'], control_params['vz'], 1)
    drone.DroneState()

# Start the client
Client_Start('192.168.14.101', 12345)
