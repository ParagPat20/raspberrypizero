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
MCU_host = '192.168.190.122'
CD2_host = '192.168.190.43'
CD4_host = '192.168.12.124'
cmd_port = 12345
ctrl_port = 54321
drone_list = []
in_line = False
wait_for_command = True
immediate_command_str = None
local_host = '192.168.190.122'

class Drone:
    
    def __init__(self, status_port, connection_string, baud=None):
        self.vehicle = connect(connection_string, baud = baud)
        self.drone_user = connection_string
        self.drone_baud = baud
        battery = self.vehicle.battery.voltage
        groundspeed = self.vehicle.groundspeed

        def send_status(self, status_port):
            global local_host
            log(local_host)
            status_socket = socket.socket()
            status_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            status_socket.bind((local_host, status_port))
            status_socket.listen(5)
            log('{} -send_status() is started!'.format(time.ctime()))
            while True:
                try:
                    client_connection, client_address = status_socket.accept() # Establish connection with client.
                    log('{} - Received follower status request from {}.'.format(time.ctime(), client_address))
                    
                    battery = str(self.vehicle.battery.voltage)
                    groundspeed = str(self.vehicle.groundspeed)
                    lat = "{:.7f}".format(self.vehicle.location.global_relative_frame.lat)
                    lon = "{:.7f}".format(self.vehicle.location.global_relative_frame.lon)
                    alt = "{:.7f}".format(self.vehicle.location.global_relative_frame.alt)
                    heading = str(self.vehicle.heading)

                    status_str = battery+','+groundspeed+','+lat+','+lon+','+alt+','+heading

                    client_connection.send(status_str.encode('utf-8'))

                    client_connection.close()

                except Exception as e:
                    log("Error: ", e)

        time.sleep(3)
        threading.Thread(target=send_status, args=(self,status_port,)).start()
        
    def reconnect(self):
        self.vehicle.exit()
        time.sleep(2)
        self = connect(self.drone_user, self.drone_baud)
        log("Reconnected Successfully")

    def arm(self, mode='GUIDED'):
        log("Arming motors")
        self.vehicle.mode = VehicleMode(mode)
        self.vehicle.armed = True
        TIMEOUT_SECONDS = 10
        start_time = time.time()
        while not self.vehicle.armed:
            log("Waiting for Arming")
            self.vehicle.armed = True
            if time.time() - start_time > TIMEOUT_SECONDS:
                break
            time.sleep(1)

        log("Vehicle Armed")
    def takeoff(self, alt = 2):
        log("Taking off!")
        self.vehicle.simple_takeoff(alt)
        start_time = time.time()
        TIMEOUT_SECONDS = 15
        while True:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            if current_altitude is not None:
                log(" Altitude: ", current_altitude)
                if current_altitude >= 1 * 0.9:
                    log("Reached target altitude")
                    break
            else:
                log("Waiting for altitude information...")
            if time.time() - start_time > TIMEOUT_SECONDS:
                break
            time.sleep(1)

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z):
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
        log(f"Drone Velocity Commands{velocity_x},{velocity_y},{velocity_z}")

        self.vehicle.send_mavlink(msg)

    def yaw(self, heading):
        current_heading = self.vehicle.heading
        log("Current Heading : ", current_heading)
        if current_heading >= 180:
            rotation = 1
        else:
            rotation = -1
        estimatedTime = heading/30.0 + 1

        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            rotation,          # param 3, direction -1 ccw, 1 cw
            0, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

        # Wait sort of time for the command to be fully executed.
        for t in range(0, int(math.ceil(estimatedTime))):
            time.sleep(1)
            log('{} - Executed yaw(heading={}) for {} seconds.'.format(time.ctime(), heading, t+1))
            self.get_vehicle_state()
            log('\n')

    def disarm(self):
        log("Disarming motors")
        self.vehicle.armed = False

        while self.vehicle.armed:
            log("Waiting for disarming...")
            self.vehicle.armed = False
            time.sleep(1)

        log("Vehicle Disarmed")

    def land(self):
        self.vehicle.mode = VehicleMode("LAND")
        log("Landing")

    def poshold(self):
        self.vehicle.mode = VehicleMode("POSHOLD")
        log("Drone currently in POSHOLD")

    def rtl(self):
        self.vehicle.mode = VehicleMode("RTL")
        log("Drone currently in RTL")

    def exit(self):
        self.vehicle.close()

    def get_vehicle_state(self):
        log('{} - Checking current Vehicle Status:'.format(time.ctime()))
        self.vehicle.battery
        
        log('     Global Location: lat={}, lon={}, alt(above sea leavel)={}'.format(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon, self.vehicle.location.global_frame.alt)) # Absolute GPS coordinate. Its lat and lon attributes are populated shortly after GPS becomes available. The alt can take several seconds longer to populate (from the barometer).
        log('     Global Location (relative altitude): lat={}, lon={}, alt(relative)={}'.format(self.vehicle.location.global_relative_frame.lat, self.vehicle.location.global_relative_frame.lon, self.vehicle.location.global_relative_frame.alt)) # GPS coordinate with relative altitude.
        log('     Local Location(NED coordinate): north={}, east={}, down={}'.format(self.vehicle.location.local_frame.north, self.vehicle.location.local_frame.east, self.vehicle.location.local_frame.down)) # North east down (NED), also known as local tangent plane (LTP)
        log('     Velocity: Vx={}, Vy={}, Vz={}'.format(self.vehicle.velocity[0], self.vehicle.velocity[1], self.vehicle.velocity[2])) #Current velocity as a three element list [ vx, vy, vz ] (in meter/sec).
        log('     GPS Info: fix_type={}, num_sat={}'.format(self.vehicle.gps_0.fix_type, self.vehicle.gps_0.satellites_visible)) # GPS Info. fix_type: 0-1, no fix; 2, 2D fix; 3, 3D fix. satellites_visible: Number of satellites visible.
        log('     Battery: voltage={}V, current={}A, level={}%'.format(self.vehicle.battery.voltage, self.vehicle.battery.current, self.vehicle.battery.level))
        log('     Heading: {} (degrees from North)'.format(self.vehicle.heading)) # Current heading in degrees(0~360), where North = 0.
        log('     Groundspeed: {} m/s'.format(self.vehicle.groundspeed)) # Current groundspeed in metres/second (double).This attribute is settable. The set value is the default target groundspeed when moving the self using simple_goto() (or other position-based movement commands).
        log('     Airspeed: {} m/s'.format(self.vehicle.airspeed)) # Current airspeed in metres/second (double).This attribute is settable. The set value is the default target airspeed when moving the self using simple_goto() (or other position-based movement commands).

    def goto(self, l, alt, groundspeed=0.7):

        log('\n')
        log('{} - Calling goto_gps_location_relative(lat={}, lon={}, alt={}, groundspeed={}).'.format(time.ctime(), l[0], l[1], alt, groundspeed))
        destination = LocationGlobalRelative(l[0], l[1], alt)
        log('{} - Before calling goto_gps_location_relative(), vehicle state is:'.format(time.ctime()))
        self.get_vehicle_state()
        current_lat = self.vehicle.location.global_relative_frame.lat
        current_lon = self.vehicle.location.global_relative_frame.lon
        current_alt = self.vehicle.location.global_relative_frame.alt
        while ((self.distance_between_two_gps_coord((current_lat,current_lon), l) >0.5) or (abs(current_alt-alt)>0.3)):
            self.vehicle.simple_goto(destination, groundspeed=groundspeed)
            time.sleep(0.5)
            current_lat = self.vehicle.location.global_relative_frame.lat
            current_lon = self.vehicle.location.global_relative_frame.lon
            current_alt = self.vehicle.location.global_relative_frame.alt
            log('{} - Horizontal distance to destination: {} m.'.format(time.ctime(), self.distance_between_two_gps_coord((current_lat,current_lon), l)))
            log('{} - Perpendicular distance to destination: {} m.'.format(time.ctime(), current_alt-alt))
        log('{} - After calling goto_gps_location_relative(), vehicle state is:'.format(time.ctime()))
        self.get_vehicle_state()

    def distance_between_two_gps_coord(self, point1, point2):
        distance = great_circle(point1, point2).meters
        return distance
    



#=============================================================================================================
    
def new_coords(original_gps_coord, displacement, rotation_degree_relative):

    vincentyDistance = geopy.distance.distance(meters = displacement)
    original_point = geopy.Point(original_gps_coord[0], original_gps_coord[1])
    new_gps_coord = vincentyDistance.destination(point=original_point, bearing=rotation_degree_relative)
    new_gps_lat = new_gps_coord.latitude
    new_gps_lon = new_gps_coord.longitude

    return round(new_gps_lat, 7), round(new_gps_lon, 7)

def cu_lo(drone):
    lat = drone.vehicle.location.global_relative_frame.lat
    lon = drone.vehicle.location.global_relative_frame.lon
    heading = drone.vehicle.heading
    return (lat,lon),heading

#==============================================================================================================

def server_receive_and_execute_immediate_command(local_host):
    global cmd_port
    global immediate_command_str
    global wait_for_command
    msg_socket = socket.socket()
    msg_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    msg_socket.bind((local_host, cmd_port))
    msg_socket.listen(5)
    log('{} - SERVER_receive_and_execute_immediate_command() is started!'.format(time.ctime()))

    while True:
        try:
            client_connection, client_address = msg_socket.accept()
            log('\n{} - Received immediate command from {}.'.format(time.ctime(), client_address))
            immediate_command_str = client_connection.recv(1024).decode()
            log('{} - Immediate command is: {}'.format(time.ctime(), immediate_command_str))
            wait_for_command = False
            ack = "Recieved Command : " + str(immediate_command_str)
            client_connection.send(ack.encode())

        except KeyboardInterrupt:
            break
        except Exception as e:
            log(f"Error: {e}")
            time.sleep(1)
        finally:
            if client_connection:
                client_connection.close()


def send(remote_host, immediate_command_str):
    global cmd_port
    # Create a socket object
    client_socket = socket.socket()
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        client_socket.connect((remote_host, cmd_port))
        client_socket.send(immediate_command_str.encode())
        log('{} - CLIENT_send_immediate_command({}, {}) is executed!'.format(time.ctime(), remote_host, immediate_command_str))
        ack = client_socket.recv(1024)
        log("ACK", ack)
    
    except socket.error as error_msg:
        log('{} - Caught exception : {}'.format(time.ctime(), error_msg))
        log('{} - CLIENT_send_immediate_command({}, {}) is not executed!'.format(time.ctime(), remote_host, immediate_command_str))
        return
    


#==============================================================================================================

def add_drone(string):
    global drone_list
    drone_list.append(string)

def remove_drone(string):
    global drone_list
    drone_list.remove(string)

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
#             log("Error: ", e)

#         finally:
#             connection.close()
#             client_socket.close()

#     # Create a socket server
#     server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     server_socket.bind((host, 8000))
#     server_socket.listen(0)

#     log("Server is listening on {}:{}".format(host, 8000))

#     while True:
#         client_socket, _ = server_socket.accept()
#         client_thread = threading.Thread(target=handle_client, args=(client_socket,))
#         client_thread.start()

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
            log('{} - Caught exception : {}'.format(time.ctime(), error_msg))
            log('{} - CLIENT_request_status({}) is not executed!'.format(time.ctime(), remote_host))
            
def chat(string):
    log(string)
    if string == 'LINECOMPLETE':
        in_line = True

def log(msg,pc_host = '192.168190.101',port = 8765):
    cli = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cli.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        cli.connect((pc_host, port))
        cli.send(str.encode(msg))
    except Exception as e:
        print("There was an error connecting to the server: " + str(e))
