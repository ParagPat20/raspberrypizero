##########** CODE FOR PI ZERO **##########

C = {'Drone': 0,'vx': 0, 'vy': 0, 'vz': 0, 'Arming': 0, 'Mode': 'GUIDED', 'Takeoff': 0}
P = 'Hello'
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import socket
import threading
import json

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

def receive_data(client_socket, data_buffer):
    while True:
        data = client_socket.recv(1024).decode()

        if not data:
            print("Connection closed by the server.")
            break

        # Append the received data to the buffer
        data_buffer.append(data)

def Control(drone, control_params):
    if control_params['Arming'] == 1:
        drone.arm(mode='STABILIZE')

    if control_params['Takeoff'] == 1:
        drone.arm(mode='GUIDED')
        drone.takeoff()
        print("Here")

def Client_Start(client_socket):
    my_drone = None
    my_drone2 = None
    drone1_init = False
    drone2_init = False

    data_buffer = ""  # Initialize a buffer to accumulate received data

    while True:
        data = client_socket.recv(1024).decode()

        if not data:
            print("Connection closed by the server.")
            break

        # Append the received data to the buffer
        data_buffer += data

        # Check if there is a complete JSON object in the buffer
        while '}' in data_buffer:
            # Extract the first JSON object from the buffer
            json_start = data_buffer.find('{')
            json_end = data_buffer.find('}') + 1
            json_data = data_buffer[json_start:json_end]

            try:
                control_params = json.loads(json_data)
            except json.JSONDecodeError as e:
                print(f"Error decoding JSON data: {e}")
                break  # Skip this JSON object and continue with the next

            # Handle the JSON data
            if control_params['Drone'] == 1:
                if not drone1_init:
                    my_drone = Drone('/dev/serial0', baudrate=115200)
                    print("Main Drone initialized")
                    drone1_init = True
                Control(my_drone, control_params)  # Pass the control parameters for the first drone

            if control_params['Drone'] == 2:
                if not drone2_init:
                    my_drone2 = Drone('0.0.0.0:14550')
                    print("Drone2 Initialized")
                    drone2_init = True
                Control(my_drone2, control_params)  # Pass the control parameters for the second drone

            if control_params['Drone'] == -1:
                Control(my_drone, control_params)  # Pass the control parameters for the first drone
                Control(my_drone2, control_params)  # Pass the control parameters for the second drone

            # Remove the processed JSON object from the buffer
            data_buffer = data_buffer[json_end:]
def process_data(data_buffer):
    my_drone = None
    my_drone2 = None
    drone1_init = False
    drone2_init = False

    while True:
        if not data_buffer:
            time.sleep(1)
            continue

        # Get the oldest data from the buffer
        data = data_buffer.pop(0)

        # Check if there is a complete JSON object in the data
        while '}' in data:
            json_start = data.find('{')
            json_end = data.find('}') + 1
            json_data = data[json_start:json_end]

            try:
                control_params = json.loads(json_data)
            except json.JSONDecodeError as e:
                print(f"Error decoding JSON data: {e}")
                break  # Skip this JSON object and continue with the next

            # Handle the JSON data
            if control_params['Drone'] == 1:
                if not drone1_init:
                    my_drone = Drone('/dev/serial0', baudrate=115200)
                    print("Main Drone initialized")
                    drone1_init = True
                Control(my_drone, control_params)  # Pass the control parameters for the first drone

            if control_params['Drone'] == 2:
                if not drone2_init:
                    my_drone2 = Drone('0.0.0.0:14550')
                    print("Drone2 Initialized")
                    drone2_init = True
                Control(my_drone2, control_params)  # Pass the control parameters for the second drone

            if control_params['Drone'] == -1:
                Control(my_drone, control_params)  # Pass the control parameters for the first drone
                Control(my_drone2, control_params)  # Pass the control parameters for the second drone

            # Remove the processed JSON object from the data
            data = data[json_end:]

# Create a socket object and connect to the server
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.14.101', 12345))
print("Connected to the server")

# Create a thread for receiving data
data_buffer = []
receive_thread = threading.Thread(target=receive_data, args=(client_socket, data_buffer))
receive_thread.start()

# Process data in a separate thread
process_thread = threading.Thread(target=process_data, args=(data_buffer,))
process_thread.start()

# Start the client
Client_Start(client_socket)
