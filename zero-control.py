##########** CODE FOR PI ZERO **##########

# Variables for drone
P = {
    0: {'Batt': 0, 'Groundspeed': 0, 'ARM': 0, 'GPS': 0, 'Altitude': 0, 'MODE': None, 'VelocityX': 0, 'VelocityY': 0, 'VelocityZ': 0},
    1: {'Batt': 0, 'Groundspeed': 0, 'ARM': 0, 'GPS': 0, 'Altitude': 0, 'MODE': None, 'VelocityX': 0, 'VelocityY': 0, 'VelocityZ': 0}
}

C = {
    0: {'vx': 0, 'vy': 0, 'vz': 0, 'Arming': 0, 'Mode': 'GUIDED', 'Takeoff': 0},
    1: {'vx': 0, 'vy': 0, 'vz': 0, 'Arming': 0, 'Mode': 'GUIDED', 'Takeoff': 0},
    'drone': None
}

from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import socket

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

    def DroneState(self):
        P['Batt'] = self.vehicle.battery.voltage  # Battery voltage
        P['Groundspeed'] = self.vehicle.groundspeed  # Groundspeed
        P['ARM'] = int(self.vehicle.armed)  # Armed status (1 for armed, 0 for disarmed)
        P['GPS'] = int(self.vehicle.gps_0.fix_type)  # GPS fix type (e.g., 3 for 3D fix)
        P['Altitude'] = self.vehicle.location.global_relative_frame.alt  # Altitude above home location
        P['MODE'] = str(self.vehicle.mode)  # Flight mode
        P['VelocityX'] = self.vehicle.velocity[0]  # Velocity in X direction (North)
        P['VelocityY'] = self.vehicle.velocity[1]  # Velocity in Y direction (East)
        P['VelocityZ'] = self.vehicle.velocity[2]  # Velocity in Z direction (Down)

def Client_Start(server_ip, server_port):
    # Create a socket object and connect to the server
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, server_port))
    print("Connected to the server")
    my_drone = None
    my_drone2 = None
    drone1_init = False
    drone2_init = False
    while True:
        p_str1 = str(P[0])
        client_socket.send(p_str1.encode())

    # Send P dictionary values for the second drone to the server
        p_str2 = str(P[1])
        client_socket.send(p_str2.encode())

    # Receive C dictionary values from the server
        c_str = client_socket.recv(2048).decode()
        control_params = eval(c_str)  # Convert the received string back to a dictionary
        c_drone = client_socket.recv(1024).decode()
        droneid = eval(c_drone) # Convert the received string back

        if droneid == 0:
            if drone1_init == False:
                my_drone = Drone('/dev/serial0',baudrate=115200)
                print("Main Drone initialized")
                drone1_init = True
            Control(my_drone, control_params[1])  # Pass the control parameters for the first drone
        
        if droneid == 1:
            if drone2_init == False:
                my_drone2 = Drone('0.0.0.0:14550')
                print("Drone2 Initialized")
                drone1_init = True
            Control(my_drone2, control_params[2])  # Pass the control parameters for the second drone

        if droneid == -1:
            Control(my_drone, control_params[1])  # Pass the control parameters for the first drone
            Control(my_drone2, control_params[2])  # Pass the control parameters for the second drone

        time.sleep(0.5)  # Adjust the sleep interval as needed


def Control(drone, control_params):

    if control_params['Mode'] == 'GUIDED' and P['ARM'] == 0 and control_params['Arming'] == 1:
        drone.arm(mode='GUIDED')

    if control_params['Takeoff'] == 1:
        drone.arm(mode='GUIDED')
        drone.takeoff()
        print("Here")
    
    if P['MODE'] != 'VehicleMode:'+control_params['Mode']:
        drone.vehicle.mode = VehicleMode(control_params['Mode'])

    drone.send_ned_velocity(control_params['vx'], control_params['vy'], control_params['vz'], 1)
    drone.DroneState()

# Start the client
Client_Start('192.168.14.101', 12345)
# Running into pi