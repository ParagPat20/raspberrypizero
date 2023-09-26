##########** CODE FOR PI ZERO **##########

C = {'Drone': 0,'vx': 0, 'vy': 0, 'vz': 0, 'Arming': 0, 'Mode': 'GUIDED', 'Takeoff': 0}
P = 'Hello'
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import socket
import threading

class Drone:
    def __init__(self, connection_string,baudrate=None):
        self.vehicle = connect(connection_string,baud = baudrate)

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle on 1 Hz cycle
        for x in range(0,duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)

    def takeoff(self):
        print("Taking off!")
        self.vehicle.simple_takeoff(1)
        while True:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            if current_altitude is not None:
                print(" Altitude: ", current_altitude)
                if current_altitude >= 1 * 0.9:
                    print("Reached target altitude")
                    break
            else:
                print("Waiting for altitude information...")
            time.sleep(1)

    def arm(self,mode='GUIDED'):
        print("Arming motors")
        self.vehicle.mode = VehicleMode(mode)
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print("Waiting for arming...")
            self.vehicle.armed = True
            time.sleep(1)

        print("Vehicle Armed")

    def disarm(self):
        print("Disarming motors")
        self.vehicle.armed = False

        while self.vehicle.armed:
            print("Waiting for disarming...")
            self.vehicle.armed = False
            time.sleep(1)

        print("Vehicle Disarmed")

    def land(self):
        self.vehicle.mode = VehicleMode("LAND")
        print("Landing")

    def exit(self):
        self.vehicle.close()
        print("Completed")
def send(client_socket):
    c_str = client_socket.recv(1024).decode()
    control_params = eval(c_str)  # Convert the received string back to a dictionary

def Client_Start(server_ip, server_port):
    # Create a socket object and connect to the server
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, server_port))
    print("Connected to the server")
    my_drone = None
    my_drone2 = None
    drone1_init = False
    drone2_init = False
    control_params = {}
    while True:

    # Receive C dictionary values from the server
        p_str1 = str(P)
        client_socket.send(p_str1.encode())
        send_thread = threading.Thread(target=send, args=(client_socket,))
        send_thread.setDaemon(True)
        send_thread.start()

        if control_params['Drone'] == 1:
            if drone1_init == False:
                my_drone = Drone('/dev/serial0',baudrate=115200)
                # my_drone = Drone('tcp:127.0.0.1:5762')
                print("Main Drone initialized")
                drone1_init = True
            Control(my_drone, control_params)  # Pass the control parameters for the first drone
        
        if control_params['Drone'] == 2:
            if drone2_init == False:
                my_drone2 = Drone('0.0.0.0:14550')
                print("Drone2 Initialized")
                drone2_init = True
            Control(my_drone2, control_params)  # Pass the control parameters for the second drone

        if control_params['Drone'] == -1:
            Control(my_drone, control_params)  # Pass the control parameters for the first drone
            Control(my_drone2, control_params)  # Pass the control parameters for the second drone

        time.sleep(1)  # Adjust the sleep interval as needed


def Control(drone, control_params):

    if control_params['Arming'] == 1:
        drone.arm(mode='STABILIZE')

    if control_params['Takeoff'] == 1:
        drone.arm(mode='GUIDED')
        drone.takeoff()
        print("Here")
    
    # if P['MODE'] != 'VehicleMode:'+control_params['Mode']:
    #     drone.vehicle.mode = VehicleMode(control_params['Mode'])

    drone.send_ned_velocity(control_params['vx'], control_params['vy'], control_params['vz'], 1)

# Start the client
Client_Start('192.168.14.101', 12345)
# Running into pi