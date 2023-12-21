## Functions
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import geopy
import geopy.distance
from geopy.distance import great_circle
import math
import threading
import zmq
# import io
# import picamera
# import struct
context = zmq.Context()  # Create a ZeroMQ context
import time
import json
import wiringpi

# use 'GPIO naming'
wiringpi.wiringPiSetupGpio()

# set #18 to be a PWM output
wiringpi.pinMode(18, wiringpi.GPIO.PWM_OUTPUT)

# set the PWM mode to milliseconds stype
wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)

# divide down clock
wiringpi.pwmSetClock(192)
wiringpi.pwmSetRange(2000)

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
    
    def __init__(self,name,connection_string, baud=None):
        self.vehicle = connect(connection_string, baud = baud)
        self.drone_user = connection_string
        self.drone_baud = baud
        self.name = name
        self.posalt = 2
        self.in_air = False
        self.no_vel_cmds = True
        self.pid_velx = {'P': 1, 'I': 0.0, 'D': 0.1}
        self.pid_vely = {'P': 1, 'I': 0.0, 'D': 0.1}
        self.pid_velz = {'P': 1, 'I': 0.0, 'D': 0.1}
        self.prev_error_velx = 0.0
        self.prev_error_vely = 0.0
        self.prev_error_velz = 0.0
        self.integral_velx = 0.0
        self.integral_vely = 0.0
        self.integral_velz = 0.0
        self.alt_ach = False
        self.prev_timestamp = time.time()

        # Create PUB socket
        self.pub_context = zmq.Context()
        self.pub_socket = self.pub_context.socket(zmq.PUB)
        self.pub_socket.bind("tcp://*:5555")  # Update with your desired PUB socket address


    def is_wifi_connected(self):
        try:
            wifi = context.socket(zmq.REQ)
            wifi.connect('tcp://192.168.207.101:8888')
            wifi.send_string("check")
            response1 = wifi.recv_string()
            wifi.send_string("check")
            response2 = wifi.recv_string()
            wifi.send_string("check")
            response3 = wifi.recv_string()
            return response1 == "Connected" or response2 == "Connected" or response3 == "Connected"

        except Exception as e:
            print(f"Error checking Wi-Fi: {e}")
            return False
        finally:
            if wifi:
                wifi.close()

    def publish_errors(self, timestamp, error_velx, error_vely, error_velz,
                       pid_output_velx, pid_output_vely, pid_output_velz):
        try:
            error_data = {
                "drone_name": self.name,
                "timestamp": timestamp,
                "error_velx": error_velx,
                "error_vely": error_vely,
                "error_velz": error_velz,
                "pid_output_velx": pid_output_velx,
                "pid_output_vely": pid_output_vely,
                "pid_output_velz": pid_output_velz
            }

            error_json = json.dumps(error_data)
            self.pub_socket.send_string(error_json)

        except Exception as e:
            log(f"Error publishing errors: {e}")

    def poshold_guided(self):
        while True:
            try:
                self.altitude = self.vehicle.location.global_relative_frame.alt
                velx = self.vehicle.velocity[0]
                vely = self.vehicle.velocity[1]
                velz = self.vehicle.velocity[2]
                if self.in_air:
                    if self.no_vel_cmds:
                        # Use PID controllers for velx and vely
                        pid_output_velx = self.calculate_pid_output(velx, self.pid_velx, 'velx')
                        pid_output_vely = self.calculate_pid_output(vely, self.pid_vely, 'vely')
                        pid_output_velz = self.calculate_pid_output(velz, self.pid_velz, 'velz')
                        if pid_output_velx > 2:
                            pid_output_velx = 2
                        if pid_output_vely > 2:
                            pid_output_vely = 2
                        if pid_output_velx < -2:
                            pid_output_velx = -2
                        if pid_output_vely < -2:
                            pid_output_vely = -2
                        if pid_output_velz > 2:
                            pid_output_velz = 2
                        if pid_output_velz > 2:
                            pid_output_velz = 2
                        self.publish_errors(time.time(), 0-velx, 0-vely, 0-velz, pid_output_velx, pid_output_vely, pid_output_velz)

                        self.send_ned_velocity_drone(pid_output_velx, pid_output_vely, pid_output_velz)

                if not self.in_air:
                    break

                time.sleep(0.1)
            except Exception as e:
                log("Poshold_Guided Error: {}".format(e))

    def calculate_pid_output(self, current_value, pid_params, axis):
        # Proportional term
        error = 0.0 - current_value
        proportional = pid_params['P'] * error
        current_timestamp = time.time()
        dt = current_timestamp - self.prev_timestamp
        self.prev_timestamp = current_timestamp
        log('dt = {}'.format(dt))

        # Integral term
        if axis == 'velx':
            self.integral_velx += error * dt  # Accumulate error over time
            integral = pid_params['I'] * self.integral_velx
            self.prev_error_velx = error
        elif axis == 'vely':
            self.integral_vely += error * dt
            integral = pid_params['I'] * self.integral_vely
            self.prev_error_vely = error
        elif axis == 'velz':
            self.integral_velz += error * dt
            integral = pid_params['I'] * self.integral_velz
            self.prev_error_velz = error
        else:
            integral = 0.0

        if not self.no_vel_cmds:
            integral = 0.0

        # Derivative term

        if axis == 'velx':
            derivative = pid_params['D'] * ((error - self.prev_error_velx) / dt)  # dt: time difference
            self.prev_error_velx = error
        elif axis == 'vely':
            derivative = pid_params['D'] * ((error - self.prev_error_vely) / dt)
            self.prev_error_vely = error
        elif axis == 'velz':
            derivative = pid_params['D'] * ((error - self.prev_error_velz) / dt)
            self.prev_error_velz = error
        else:
            derivative = 0.0

        # Summing up all terms
        pid_output = proportional + integral + derivative

        return pid_output

    def security(self):
        self.altitude = self.vehicle.location.global_relative_frame.alt
        self.battery = self.vehicle.battery.voltage
        log(f"{self.name}'s Security checkup started!")
        # threading.Thread(target=self.poshold_guided).start()

        while True:
            try:
                self.altitude = self.vehicle.location.global_relative_frame.alt
                self.battery = self.vehicle.battery.voltage
                self.wifi_status = self.is_wifi_connected()
                self.mode = self.vehicle.mode
                velx = self.vehicle.velocity[0]
                vely = self.vehicle.velocity[1]
                log('sec {} PosAlt: {}m \n      Current altitude : {}m\n      Current Battery {}V\n      Alt Difference {}\n      Wifi Status {}\n{}'.format(self.name,self.posalt, self.altitude, self.battery, self.altitude - self.posalt, str(self.wifi_status), str(self.mode)))
                coordlat = str(self.vehicle.location.global_relative_frame.lat)
                coordlon = str(self.vehicle.location.global_relative_frame.lon)
                log("lat {}".format(coordlat))
                log("lon {}".format(coordlon))
                if not self.wifi_status:
                    print("{} Wi-Fi connection lost! Initiating landing.".format(self.name))
                    self.land()
                    self.disarm()
                if self.altitude > 5:
                    log("sec {} Altitude greater than 5 meters! Initiating landing.".format(self.name))
                    self.land()
                if self.battery < 10.5:
                    log("sec {} Battery LOW, Landing".format(self.name))
                    self.land()
                time.sleep(4)
            except Exception as e:
                log("sec {} Security Error : {}".format(self.name,e))
                pass

    # def send_status(self, local_host, status_port):
    #     def handle_clients(client_connection, client_address):
    #         log('{} - Received follower status request from {}.'.format(time.ctime(), client_address))
            
    #         battery = str(self.vehicle.battery.voltage)
    #         groundspeed = str(self.vehicle.groundspeed)
    #         lat = "{:.7f}".format(self.vehicle.location.global_relative_frame.lat)
    #         lon = "{:.7f}".format(self.vehicle.location.global_relative_frame.lon)
    #         alt = "{:.7f}".format(self.vehicle.location.global_relative_frame.alt)
    #         heading = str(self.vehicle.heading)

    #         status_str = battery+','+groundspeed+','+lat+','+lon+','+alt+','+heading

    #         client_connection.send(status_str.encode('utf-8'))
    #         client_connection.close()


    #     status_socket = socket.socket()
    #     status_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    #     status_socket.bind((local_host, status_port))
    #     status_socket.listen(10)
    #     log('{} -send_status() is started!'.format(time.ctime()))

    #     while True:
    #         try:
    #             client_connection, client_address = status_socket.accept() # Establish connection with client.

    #             handle = threading.Thread(target=handle_clients, args=(client_connection, client_address,))
    #             handle.start()

    #         except Exception as e:
    #             log("Error: sending battery...")

        
    def reconnect(self):
        try:
            self.vehicle.close()
            self = None
            time.sleep(2)
            log("Reconnected Successfully")
        except Exception as e:
            log(f"Error during reconnection: {e}")

    def arm(self, mode='GUIDED'):
        try:
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
        except Exception as e:
            log(f"Error during arming: {e}")

    def takeoff(self, alt=2):
        try:
            self.arm()
            log("Taking off!")
            self.vehicle.simple_takeoff(alt)
            start_time = time.time()
            TIMEOUT_SECONDS = 15
            self.posalt = alt
            while True:
                current_altitude = self.vehicle.location.global_relative_frame.alt
                if current_altitude is not None:
                    log(" Altitude: {}".format(current_altitude))
                    if current_altitude >= 1 * 0.9:
                        log("Reached target altitude")
                        break
                else:
                    log("Waiting for altitude information...")
                if time.time() - start_time > TIMEOUT_SECONDS:
                    break
                time.sleep(1)
            self.in_air = True
            self.alt_ach = False
        except Exception as e:
            log(f"Error during takeoff: {e}")

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

            self.vehicle.send_mavlink(msg)
            log("Drone Velocity : {}, {}, {}".format(velocity_x,velocity_y,velocity_z))

        except Exception as e:
            log(f"Error sending velocity commands: {e}")

    def send_ned_velocity(self, x, y, z, duration = None):
        self.no_vel_cmds = False
        if duration:
            for i in range(0,duration):
                self.send_ned_velocity_drone(x,y,z)
                log(i)
                time.sleep(1)

            self.send_ned_velocity_drone(0,0,0)
            time.sleep(1)
            self.no_vel_cmds = True
            
        else:
            self.send_ned_velocity_drone(x,y,z)
            time.sleep(1)
            self.no_vel_cmds = True

    def yaw(self, heading):
        try:
            current_heading = self.vehicle.heading
            log("Current Heading : {}".format(current_heading))
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
                log('{} - Executed yaw(heading={}) for {} seconds.'.format(time.ctime(), heading, t + 1))
        except Exception as e:
            log(f"Error during yaw command: {e}")

    def servo(self,cmd):
        delay_period = 0.01
        close = 'close'
        open = 'open'
        try:
            if cmd == 'close' or cmd == close:
                for pulse in range(50, 250, 1):
                    wiringpi.pwmWrite(18, pulse)
                    time.sleep(delay_period)
            if cmd == 'open' or cmd == open:
                for pulse in range(250, 50, -1):
                    wiringpi.pwmWrite(18, pulse)
                    time.sleep(delay_period)
            log('setting servo to {}'.format(cmd))
        except Exception as e:
            log(f"Error during servo command: {e}")

    def disarm(self):
        try:
            log("Disarming motors")
            self.vehicle.armed = False

            while self.vehicle.armed:
                log("Waiting for disarming...")
                self.vehicle.armed = False
                time.sleep(1)

            log("Vehicle Disarmed")
        except Exception as e:
            log(f"Error during disarming: {e}")

    def land(self):
        try:
            self.vehicle.mode = VehicleMode("LAND")
            log("Landing")
            self.in_air = False
        except Exception as e:
            log(f"Error during landing: {e}")

    def poshold(self):
        try:
            self.vehicle.mode = VehicleMode("POSHOLD")
            log("Drone currently in POSHOLD")
        except Exception as e:
            log(f"Error during POSHOLD mode setting: {e}")

    def rtl(self):
        try:
            self.vehicle.mode = VehicleMode("RTL")
            log("Drone currently in RTL")
            self.in_air = False
        except Exception as e:
            log(f"Error during RTL mode setting: {e}")

    def exit(self):
        try:
            self.vehicle.close()
        except Exception as e:
            log(f"Error during vehicle exit: {e}")

    def get_vehicle_state(self):
        try:
            log_msg = (
                '{} - Checking current Vehicle Status:\n'
                '     Global Location: lat={}, lon={}, alt(above sea level)={}\n'
                '     Global Location (relative altitude): lat={}, lon={}, alt(relative)={}\n'
                '     Local Location(NED coordinate): north={}, east={}, down={}\n'
                '     Velocity: Vx={}, Vy={}, Vz={}\n'
                '     GPS Info: fix_type={}, num_sat={}\n'
                '     Battery: voltage={}V, current={}A, level={}%\n'
                '     Heading: {} (degrees from North)\n'
                '     Groundspeed: {} m/s\n'
                '     Airspeed: {} m/s'
            ).format(
                time.ctime(),
                self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon,
                self.vehicle.location.global_frame.alt,
                self.vehicle.location.global_relative_frame.lat, self.vehicle.location.global_relative_frame.lon,
                self.vehicle.location.global_relative_frame.alt,
                self.vehicle.location.local_frame.north, self.vehicle.location.local_frame.east,
                self.vehicle.location.local_frame.down,
                self.vehicle.velocity[0], self.vehicle.velocity[1], self.vehicle.velocity[2],
                self.vehicle.gps_0.fix_type, self.vehicle.gps_0.satellites_visible,
                self.vehicle.battery.voltage, self.vehicle.battery.current, self.vehicle.battery.level,
                self.vehicle.heading, self.vehicle.groundspeed, self.vehicle.airspeed
            )

            log(log_msg)


        except Exception as e:
            log(f"Error getting vehicle state: {e}")

    def goto(self, l, alt, groundspeed=0.7):
        try:
            log('\n')
            log('{} - Calling goto_gps_location_relative(lat={}, lon={}, alt={}, groundspeed={}).'.format(
                time.ctime(), l[0], l[1], alt, groundspeed))
            destination = LocationGlobalRelative(l[0], l[1], alt)
            log('{} - Before calling goto_gps_location_relative(), vehicle state is:'.format(time.ctime()))
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
                log('{} - Horizontal distance to destination: {} m.'.format(time.ctime(),
                                                                             self.distance_between_two_gps_coord(
                                                                                 (current_lat, current_lon), l)))
                log('{} - Perpendicular distance to destination: {} m.'.format(time.ctime(),
                                                                                 current_alt - alt))
            log('{} - After calling goto_gps_location_relative(), vehicle state is:'.format(time.ctime()))
            self.get_vehicle_state()
        except Exception as e:
            log(f"Error during goto command: {e}")

    def distance_between_two_gps_coord(self, point1, point2):
        try:
            distance = great_circle(point1, point2).meters
            return distance
        except Exception as e:
            log(f"Error calculating distance between two GPS coordinates: {e}")


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
        log(f"Error in calculating new coordinates: {e}")

def cu_lo(drone):
    try:
        lat = drone.vehicle.location.global_relative_frame.lat
        lon = drone.vehicle.location.global_relative_frame.lon
        heading = drone.vehicle.heading
        return (lat, lon), heading
    except Exception as e:
        log(f"Error in getting current location: {e}")
        return (0.0, 0.0), 0.0  # Returning default values in case of an error
    
def check_distance(d1,d2):
    try:
        log("First drone's current location{}".format(cu_lo(d1)))
        log("Second drone's current location{}".format(cu_lo(d2)))
        distance = d1.distance_between_two_gps_coord(cu_lo(d1)[0],cu_lo(d2)[0])
        log("Distance between those drones is {} meters".format(distance))
        
    except Exception as e:
        log(f"Error in check_distance: {e}")



#==============================================================================================================

connected_hosts = set()
clients = {}
pc = '192.168.207.101'

import random

def send(host, immediate_command_str):
    global connected_hosts
    global clients

    if host not in connected_hosts:
        context = zmq.Context()
        socket1 = context.socket(zmq.PUSH)
        socket1.connect(f"tcp://{host}:12345")
        socket2 = context.socket(zmq.PUSH)
        socket2.connect(f"tcp://{host}:12345")
        socket3 = context.socket(zmq.PUSH)
        socket3.connect(f"tcp://{host}:12345")
        clients[host] = [socket1,socket2,socket3]
        connected_hosts.add(host)
    immediate_command_str = str(immediate_command_str)
    random_socket = random.choice(clients[host])
    random_socket.send_string(immediate_command_str)

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
#     server_socket.listen(2)

#     log("Server is listening on {}:{}".format(host, 8000))

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
        log(f"Error adding drone: {e}")

def remove_drone(string):
    try:
        global drone_list
        drone_list.remove(string)
    except Exception as e:
        log(f"Error removing drone: {e}")


# def recv_status(remote_host,status_port):
        
#         client_socket = socket.socket()
#         client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#         try:
#             client_socket.connect((remote_host, status_port))
#             status_msg_str = client_socket.recv(1024).decode('utf-8')
#             battery ,gs, lat, lon, alt, heading = status_msg_str.split(',')
#             lat = float(lat)
#             lon = float(lon)
#             alt = float(alt)
#             heading = float(heading)

#             return (lat,lon),alt,heading
#         except socket.error as error_msg:
#             log('{} - Caught exception : {}'.format(time.ctime(), error_msg))
#             log('{} - CLIENT_request_status({}) is not executed!'.format(time.ctime(), remote_host))
            
def chat(string):
    try:
        print(string)

    except Exception as e:
        log(f"Error in chat function: {e}")


import random

def log(immediate_command_str):
    global connected_hosts
    global clients
    host = pc

    if host not in connected_hosts:
        context = zmq.Context()
        socket1 = context.socket(zmq.PUSH)
        socket1.connect(f"tcp://{host}:5556")
        socket2 = context.socket(zmq.PUSH)
        socket2.connect(f"tcp://{host}:5556")
        socket3 = context.socket(zmq.PUSH)
        socket3.connect(f"tcp://{host}:5556")
        socket4 = context.socket(zmq.PUSH)
        socket4.connect(f"tcp://{host}:5556")
        socket5 = context.socket(zmq.PUSH)
        socket5.connect(f"tcp://{host}:5556")
        socket6 = context.socket(zmq.PUSH)
        socket6.connect(f"tcp://{host}:5556")
        clients[host] = [socket1,socket2,socket3,socket4,socket5,socket6]
        connected_hosts.add(host)

    random_socket = random.choice(clients[host])
    immediate_command_str = str(immediate_command_str)
    random_socket.send_string(immediate_command_str)
