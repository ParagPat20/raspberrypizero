## Functions
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import geopy
import geopy.distance
from geopy.distance import great_circle
import math
import threading
import socket
# import io
# import picamera
# import struct

'''
global variables
status_port = 60001,60002
cmd_port
local_host
'''
d1 = None
d2 = None
selected_drone = None
MCU_host = '192.168.207.122'
CD1_host = '192.168.207.43'
CD2_host = '192.168.207.225'
cmd_port = 12345
ctrl_port = 54321
drone_list = []
wait_for_command = True
immediate_command_str = None

class Drone:
    
    def __init__(self,connection_string, baud=None):
        self.vehicle = connect(connection_string, baud = baud)
        self.drone_user = connection_string
        self.drone_baud = baud

    def send_status(self, local_host, status_port):
        def handle_clients(client_connection, client_address):
            print('{} - Received follower status request from {}.'.format(time.ctime(), client_address))
            
            battery = str(self.vehicle.battery.voltage)
            groundspeed = str(self.vehicle.groundspeed)
            lat = "{:.7f}".format(self.vehicle.location.global_relative_frame.lat)
            lon = "{:.7f}".format(self.vehicle.location.global_relative_frame.lon)
            alt = "{:.7f}".format(self.vehicle.location.global_relative_frame.alt)
            heading = str(self.vehicle.heading)

            status_str = battery+','+groundspeed+','+lat+','+lon+','+alt+','+heading

            client_connection.send(status_str.encode('utf-8'))
            client_connection.close()


        status_socket = socket.socket()
        status_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        status_socket.bind((local_host, status_port))
        status_socket.listen(5)
        print('{} -send_status() is started!'.format(time.ctime()))

        while True:
            try:
                client_connection, client_address = status_socket.accept() # Establish connection with client.

                handle = threading.Thread(target=handle_clients, args=(client_connection, client_address,))
                handle.start()

            except Exception as e:
                print("Error: sending battery...")

        
    def reconnect(self):
        try:
            self.vehicle.close()
            self = None
            time.sleep(2)
            print("Reconnected Successfully")
        except Exception as e:
            print(f"Error during reconnection: {e}")

    def arm(self, mode='GUIDED'):
        try:
            print("Arming motors")
            self.vehicle.mode = VehicleMode(mode)
            self.vehicle.armed = True
            TIMEOUT_SECONDS = 10
            start_time = time.time()
            while not self.vehicle.armed:
                print("Waiting for Arming")
                self.vehicle.armed = True
                if time.time() - start_time > TIMEOUT_SECONDS:
                    break
                time.sleep(1)

            print("Vehicle Armed")
        except Exception as e:
            print(f"Error during arming: {e}")

    def takeoff(self, alt=2):
        try:
            self.arm()
            print("Taking off!")
            self.vehicle.simple_takeoff(alt)
            start_time = time.time()
            TIMEOUT_SECONDS = 15
            while True:
                current_altitude = self.vehicle.location.global_relative_frame.alt
                if current_altitude is not None:
                    print(" Altitude: {}".format(current_altitude))
                    if current_altitude >= 1 * 0.9:
                        print("Reached target altitude")
                        break
                else:
                    print("Waiting for altitude information...")
                if time.time() - start_time > TIMEOUT_SECONDS:
                    break
                time.sleep(1)
        except Exception as e:
            print(f"Error during takeoff: {e}")

    def send_ned_velocity_drone(self, velocity_x, velocity_y, velocity_z):
        try:
            velocity_x = float(velocity_x)
            velocity_y = float(velocity_y)
            velocity_z = float(velocity_z)

            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,  # time_boot_ms (not used)
                0, 0,  # target system, target component
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
                0b0000111111000111,  # type_mask (only speeds enabled)
                0, 0, 0,  # x, y, z positions (not used)
                velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
                0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
                0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
            print(f"Drone Velocity Commands{velocity_x},{velocity_y},{velocity_z}")

            self.vehicle.send_mavlink(msg)


        except Exception as e:
            print(f"Error sending velocity commands: {e}")

    def send_ned_velocity(self, x, y, z, duration = None):
        if duration:
            for i in range(0,duration):
                self.send_ned_velocity_drone(x,y,z)
                print(i)
                time.sleep(1)

            self.send_ned_velocity_drone(0,0,0)
            
        else:
            self.send_ned_velocity_drone(x,y,z)
            time.sleep(0.5)
            self.send_ned_velocity_drone(0,0,0)

    def yaw(self, heading):
        try:
            current_heading = self.vehicle.heading
            print("Current Heading : {}".format(current_heading))
            if heading - current_heading <= 0:
                rotation = 1
            else:
                rotation = -1
            estimatedTime = heading / 30.0 + 1

            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,  # target system, target component
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
                0,  # confirmation
                heading,  # param 1, yaw in degrees
                0,  # param 2, yaw speed deg/s
                rotation,  # param 3, direction -1 ccw, 1 cw
                0,  # param 4, relative offset 1, absolute angle 0
                0, 0, 0)  # param 5 ~ 7 not used
            # send command to vehicle
            self.vehicle.send_mavlink(msg)

            # Wait sort of time for the command to be fully executed.
            for t in range(0, int(math.ceil(estimatedTime))):
                time.sleep(1)
                print('{} - Executed yaw(heading={}) for {} seconds.'.format(time.ctime(), heading, t + 1))
                self.get_vehicle_state()
                print('\n')
        except Exception as e:
            print(f"Error during yaw command: {e}")

    def disarm(self):
        try:
            print("Disarming motors")
            self.vehicle.armed = False

            while self.vehicle.armed:
                print("Waiting for disarming...")
                self.vehicle.armed = False
                time.sleep(1)

            print("Vehicle Disarmed")
        except Exception as e:
            print(f"Error during disarming: {e}")

    def land(self):
        try:
            self.vehicle.mode = VehicleMode("LAND")
            print("Landing")
        except Exception as e:
            print(f"Error during landing: {e}")

    def poshold(self):
        try:
            self.vehicle.mode = VehicleMode("POSHOLD")
            print("Drone currently in POSHOLD")
        except Exception as e:
            print(f"Error during POSHOLD mode setting: {e}")

    def rtl(self):
        try:
            self.vehicle.mode = VehicleMode("RTL")
            print("Drone currently in RTL")
        except Exception as e:
            print(f"Error during RTL mode setting: {e}")

    def exit(self):
        try:
            self.vehicle.close()
        except Exception as e:
            print(f"Error during vehicle exit: {e}")

    def get_vehicle_state(self):
        try:
            print('{} - Checking current Vehicle Status:'.format(time.ctime()))
            self.vehicle.battery

            print('     Global Location: lat={}, lon={}, alt(above sea leavel)={}'.format(
                self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon,
                self.vehicle.location.global_frame.alt))
            print('     Global Location (relative altitude): lat={}, lon={}, alt(relative)={}'.format(
                self.vehicle.location.global_relative_frame.lat, self.vehicle.location.global_relative_frame.lon,
                self.vehicle.location.global_relative_frame.alt))
            print('     Local Location(NED coordinate): north={}, east={}, down={}'.format(
                self.vehicle.location.local_frame.north, self.vehicle.location.local_frame.east,
                self.vehicle.location.local_frame.down))
            print('     Velocity: Vx={}, Vy={}, Vz={}'.format(self.vehicle.velocity[0], self.vehicle.velocity[1],
                                                            self.vehicle.velocity[2]))
            print('     GPS Info: fix_type={}, num_sat={}'.format(self.vehicle.gps_0.fix_type,
                                                                self.vehicle.gps_0.satellites_visible))
            print('     Battery: voltage={}V, current={}A, level={}%'.format(self.vehicle.battery.voltage,
                                                                           self.vehicle.battery.current,
                                                                           self.vehicle.battery.level))
            print('     Heading: {} (degrees from North)'.format(self.vehicle.heading))
            print('     Groundspeed: {} m/s'.format(self.vehicle.groundspeed))
            print('     Airspeed: {} m/s'.format(self.vehicle.airspeed))

        except Exception as e:
            print(f"Error getting vehicle state: {e}")

    def goto(self, l, alt, groundspeed=0.7):
        try:
            print('\n')
            print('{} - Calling goto_gps_location_relative(lat={}, lon={}, alt={}, groundspeed={}).'.format(
                time.ctime(), l[0], l[1], alt, groundspeed))
            destination = LocationGlobalRelative(l[0], l[1], alt)
            print('{} - Before calling goto_gps_location_relative(), vehicle state is:'.format(time.ctime()))
            self.get_vehicle_state()
            current_lat = self.vehicle.location.global_relative_frame.lat
            current_lon = self.vehicle.location.global_relative_frame.lon
            current_alt = self.vehicle.location.global_relative_frame.alt
            while ((self.distance_between_two_gps_coord((current_lat, current_lon), l) > 0.5) or (
                    abs(current_alt - alt) > 0.3)):
                self.vehicle.simple_goto(destination, groundspeed=groundspeed)
                time.sleep(0.5)
                current_lat = self.vehicle.location.global_relative_frame.lat
                current_lon = self.vehicle.location.global_relative_frame.lon
                current_alt = self.vehicle.location.global_relative_frame.alt
                print('{} - Horizontal distance to destination: {} m.'.format(time.ctime(),
                                                                             self.distance_between_two_gps_coord(
                                                                                 (current_lat, current_lon), l)))
                print('{} - Perpendicular distance to destination: {} m.'.format(time.ctime(),
                                                                                 current_alt - alt))
            print('{} - After calling goto_gps_location_relative(), vehicle state is:'.format(time.ctime()))
            self.get_vehicle_state()
        except Exception as e:
            print(f"Error during goto command: {e}")

    def distance_between_two_gps_coord(self, point1, point2):
        try:
            distance = great_circle(point1, point2).meters
            return distance
        except Exception as e:
            print(f"Error calculating distance between two GPS coordinates: {e}")


#=============================================================================================================
    
def new_coords(original_gps_coord, displacement, rotation_degree_relative):
    try:
        vincentyDistance = geopy.distance.distance(meters=displacement)
        original_point = geopy.Point(original_gps_coord[0], original_gps_coord[1])
        new_gps_coord = vincentyDistance.destination(point=original_point, bearing=rotation_degree_relative)
        new_gps_lat = new_gps_coord.latitude
        new_gps_lon = new_gps_coord.longitude

        return round(new_gps_lat, 7), round(new_gps_lon, 7)
    except Exception as e:
        print(f"Error in calculating new coordinates: {e}")

def cu_lo(drone):
    try:
        lat = drone.vehicle.location.global_relative_frame.lat
        lon = drone.vehicle.location.global_relative_frame.lon
        heading = drone.vehicle.heading
        return (lat, lon), heading
    except Exception as e:
        print(f"Error in getting current location: {e}")
        return (0.0, 0.0), 0.0  # Returning default values in case of an error


#==============================================================================================================

def send(remote_host, immediate_command_str):
    global cmd_port
    # Create a socket object
    client_socket = socket.socket()
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        client_socket.connect((remote_host, cmd_port))
        client_socket.send(immediate_command_str.encode())
        print('{} - CLIENT_send_immediate_command({}, {}) is executed!'.format(time.ctime(), remote_host, immediate_command_str))

    except socket.error as error_msg:
        print('{} - Caught exception : {}'.format(time.ctime(), error_msg))
        print('{} - CLIENT_send_immediate_command({}, {}) is not executed!'.format(time.ctime(), remote_host, immediate_command_str))
        return
    finally:
        if client_socket:
            client_socket.close()
    
# def camera_stream_server(host):
#     def handle_client(client_socket):
#         connection = client_socket.makefile('wb')

#         try:
#             with picamera.PiCamera() as camera:
#                 camera.resolution = (640, 480)  # Adjust resolution as needed
#                 camera.framerate = 30  # Adjust frame rate as needed

#                 # Start capturing and sending the video feed
#                 time.sleep(2)  # Give the camera some time to warm up
#                 stream = io.BytesIO()
#                 for _ in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
#                     stream.seek(0)
#                     image_data = stream.read()

#                     # Send the image size to the client
#                     connection.write(struct.pack('<L', len(image_data)))
#                     connection.flush()

#                     # Send the image data to the client
#                     connection.write(image_data)
#                     stream.seek(0)
#                     stream.truncate()
#         except Exception as e:
#             print("Error: ", e)

#         finally:
#             connection.close()
#             client_socket.close()

#     # Create a socket server
#     server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     server_socket.bind((host, 8000))
#     server_socket.listen(2)

#     print("Server is listening on {}:{}".format(host, 8000))

#     while True:
#         client_socket, _ = server_socket.accept()
#         client_thread = threading.Thread(target=handle_client, args=(client_socket,))
#         client_thread.start()

#==============================================================================================================

def add_drone(string):
    try:
        global drone_list
        drone_list.append(string)
    except Exception as e:
        print(f"Error adding drone: {e}")

def remove_drone(string):
    try:
        global drone_list
        drone_list.remove(string)
    except Exception as e:
        print(f"Error removing drone: {e}")


def recv_status(remote_host,status_port):
        
        client_socket = socket.socket()
        client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            client_socket.connect((remote_host, status_port))
            status_msg_str = client_socket.recv(1024).decode('utf-8')
            battery ,gs, lat, lon, alt, heading = status_msg_str.split(',')
            lat = float(lat)
            lon = float(lon)
            alt = float(alt)
            heading = float(heading)

            return (lat,lon),alt,heading
        except socket.error as error_msg:
            print('{} - Caught exception : {}'.format(time.ctime(), error_msg))
            print('{} - CLIENT_request_status({}) is not executed!'.format(time.ctime(), remote_host))
            
def chat(string):
    try:
        print(string)

    except Exception as e:
        print(f"Error in chat function: {e}")

def log(immediate_command_str):
    print('thread starting')
    client_socket = socket.socket()
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    print('connecting')
    client_socket.connect(('192.168.207.101',12345))
    print('connected,sending')
    client_socket.send(immediate_command_str.encode())
    print('sent,close')
    client_socket.close()

def check_distance(d1,d2):
    try:
        print("First drone's current location{}".format(cu_lo(d1)))
        print("Second drone's current location{}".format(cu_lo(d2)))
        distance = d1.distance_between_two_gps_coord(cu_lo(d1)[0],cu_lo(d2)[0])
        print("Distance between those drones is {} meters".format(distance))
        
    except Exception as e:
        print(f"Error in check_distance: {e}")


# import sys

# class LogStream:
#     def __init__(self):
#         self.buffer = ""

#     def write(self, data):
#         self.buffer += data
#         while "\n" in self.buffer:
#             line, self.buffer = self.buffer.split("\n", 1)
#             log(line)

#     def flush(self):
#         pass

# log_stream = LogStream()
# sys.stdout = log_stream
# sys.stderr = log_stream