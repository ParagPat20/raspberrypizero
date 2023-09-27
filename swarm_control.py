##########** CODE FOR PI ZERO **##########

C = {
    'Drone': 0,
    'vx': 0, 
    'vy': 0, 
    'vz': 0, 
    'Arming': 0, 
    'Mode': 'GUIDED',
    'Takeoff': 0
    }

P = {
    'MCU':{
        'Batt' : 0,
        'Groundspeed': 0,
        'ARM': 0,
        'GPS': 0,
        'Altitude': 0,
        'MODE': None,
        'VelocityX': 0,
        'VelocityY': 0,
        'VelocityZ': 0
        },
    'Drone':{
        'Batt' : 0,
        'Groundspeed': 0,
        'ARM': 0,
        'GPS': 0,
        'Altitude': 0,
        'MODE': None,
        'VelocityX': 0,
        'VelocityY': 0,
        'VelocityZ': 0
    }
    }

from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import socket
import threading
import pickle

class Drone:
    global C,P
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
            time.sleep(0.5)

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

    def arm(self,mode):
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


def Client_Start(server_ip, server_port):
    global C,P
    # Create a socket object and connect to the server
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, server_port))
    print("Connected to the server")
    my_drone = None
    my_drone2 = None
    drone1_init = False
    drone2_init = False
    while True:
        # Receive C dictionary values from the server
        c_pickle = client_socket.recv(1024)
        if not c_pickle:
            break
        C = pickle.loads(c_pickle)

        if C['Drone'] == 1 or C['Drone'] == -1:
            if not drone1_init:
                my_drone = Drone('/dev/serial0', baudrate=115200)
                # my_drone = Drone('tcp:127.0.0.1:5762')
                print("Main Drone initialized")
                ctrl = threading.Thread(target=Control1,args=(my_drone,))
                ctrl.start()
                time.sleep(0.5)
                drone1_init = True
        if drone1_init:
            if my_drone.vehicle.battery.voltage is not None:
                P['MCU']['Batt'] = my_drone.vehicle.battery.voltage  # Battery voltage
            else:
                P['MCU']['Batt'] = 0
            P['MCU']['Groundspeed'] = my_drone.vehicle.groundspeed  # Groundspeed
            P['MCU']['ARM'] = int(my_drone.vehicle.armed)  # Armed status (1 for armed, 0 for disarmed)
            if my_drone.vehicle.gps_0 is not None:
                P['MCU']['GPS'] = int(my_drone.vehicle.gps_0.fix_type)  # GPS fix type (e.g., 3 for 3D fix)
            else:
                P['MCU']['GPS'] = 0
            P['MCU']['Altitude'] = my_drone.vehicle.location.global_relative_frame.alt  # Altitude above home location
            P['MCU']['MODE'] = str(my_drone.vehicle.mode)  # Flight mode
            P['MCU']['VelocityX'] = my_drone.vehicle.velocity[0]  # Velocity in X direction (North)
            P['MCU']['VelocityY'] = my_drone.vehicle.velocity[1]  # Velocity in Y direction (East)
            P['MCU']['VelocityZ'] = my_drone.vehicle.velocity[2]  # Velocity in Z direction (Down)
        
        if C['Drone'] == 2 or C['Drone'] == -1:
            if not drone2_init:
                my_drone2 = Drone('0.0.0.0:14550')
                # my_drone2 = Drone('tcp:127.0.0.1:5772')
                print("Drone2 Initialized")
                ctrl1 = threading.Thread(target=Control2,args=(my_drone2,))
                ctrl1.start()
                time.sleep(0.5)
                drone2_init = True
        if drone2_init:
            if my_drone2.vehicle.battery.voltage is not None:
                P['Drone']['Batt'] = my_drone.vehicle.battery.voltage  # Battery voltage
            else:
                P['Drone']['Batt'] = 0
            P['Drone']['Groundspeed'] = my_drone2.vehicle.groundspeed  # Groundspeed
            P['Drone']['ARM'] = int(my_drone2.vehicle.armed)  # Armed status (1 for armed, 0 for disarmed)
            if my_drone.vehicle.gps_0 is not None:
                P['Drone']['GPS'] = int(my_drone.vehicle.gps_0.fix_type)  # GPS fix type (e.g., 3 for 3D fix)
            else:
                P['Drone']['GPS'] = 0
            P['Drone']['Altitude'] = my_drone2.vehicle.location.global_relative_frame.alt  # Altitude above home location
            P['Drone']['MODE'] = str(my_drone2.vehicle.mode)  # Flight mode
            P['Drone']['VelocityX'] = my_drone2.vehicle.velocity[0]  # Velocity in X direction (North)
            P['Drone']['VelocityY'] = my_drone2.vehicle.velocity[1]  # Velocity in Y direction (East)
            P['Drone']['VelocityZ'] = my_drone2.vehicle.velocity[2]  # Velocity in Z direction (Down)
        p_pickle = pickle.dumps(P)
        client_socket.send(p_pickle)


def Control1(drone):
    global C,P
    while True:
        if C['Drone'] == 1 or C['Drone'] == -1:
            if C['Arming'] == 1:
                drone.arm(C['Mode'])

            if C['Takeoff'] == 1:
                drone.arm(C['Mode'])
                drone.takeoff()
                print("Here")

            if P['MCU']['MODE'] != 'VehicleMode:'+C['Mode']:
                drone.vehicle.mode = VehicleMode(C['Mode'])

            drone.send_ned_velocity(C['vx'], C['vy'], C['vz'], 1)

def Control2(drone):
    global C,P
    while True:
        if C['Drone'] == 2 or C['Drone'] == -1:
            if C['Arming'] == 1:
                drone.arm(C['Mode'])

            if C['Takeoff'] == 1:
                drone.arm(C['Mode'])
                drone.takeoff()
                print("Here")

            if P['Drone']['MODE'] != 'VehicleMode:'+C['Mode']:
                drone.vehicle.mode = VehicleMode(C['Mode'])

            drone.send_ned_velocity(C['vx'], C['vy'], C['vz'], 1)

# Start the client
Client_Start('192.168.14.101', 12345)
# Running into pi