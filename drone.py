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
import io
import picamera
import struct
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
MCU_host = '172.217.28.101'
CD1_host = '172.217.28.102'
CD2_host = '172.217.28.103'
CD3_host = CD2_host
cmd_port = 12345
ctrl_port = 54321
drone_list = []
wait_for_command = True
immediate_command_str = None
missions = {}
poller = zmq.Poller
wifi_status = True

class Drone:
    
    def __init__(self,name,connection_string, baud=None):
        self.vehicle = connect(connection_string, baud = baud)
        self.drone_user = connection_string
        self.drone_baud = baud
        self.name = name
        self.posalt = 2
        self.in_air = False
        self.no_vel_cmds = True
        self.pid_velx = {'P': 0.8, 'I': 0.0, 'D': 0.1}
        self.pid_vely = {'P': 0.8, 'I': 0.0, 'D': 0.1}
        self.pid_velz = {'P': 0.5, 'I': 0.0, 'D': 0.1}
        self.prev_error_velx = 0.0
        self.prev_error_vely = 0.0
        self.prev_error_velz = 0.0
        self.integral_velx = 0.0
        self.integral_vely = 0.0
        self.integral_velz = 0.0
        self.alt_ach = False
        self.prev_timestamp = time.time()
        self.wifi_status = True


    def is_wifi_connected(self):
        global wifi_status
        while True:
            try:
                wifi = context.socket(zmq.REQ)
                wifi.connect('tcp://172.217.28.100:8888')

                wifi.send_string(self.name)
                wifi.setsockopt(zmq.RCVTIMEO, 5000)  # Set 3-second timeout for response

                try:
                    response = wifi.recv_string()
                    if response == "Connected":
                        self.wifi_status = True
                        wifi_status = self.wifi_status
                    else:
                        self.wifi_status = False
                        wifi_status = self.wifi_status
                    wifi.close()
                except zmq.Again:  # Timeout occurred
                    self.wifi_status = False
                    wifi_status = self.wifi_status
                    print("Waiting for new connection to be established")
                    wifi.close()
                    wifi = context.socket(zmq.REQ)
                    wifi.connect('tcp://172.217.28.100:8888')

                    wifi.send_string("check")
                    wifi.setsockopt(zmq.RCVTIMEO, 10000)

                time.sleep(2)

            except zmq.ZMQError as e:
                print(f"ZMQ Error: {e}")
                self.wifi_status = False

            except Exception as e:
                print(f"General Error: {e}")
                self.wifi_status=False

            finally:
                if wifi:
                    wifi.close()

    def poshold_guided(self):
        while True:
            try:
                self.altitude = self.vehicle.location.global_relative_frame.alt
                velx = self.vehicle.velocity[0]
                vely = self.vehicle.velocity[1]
                velz = self.vehicle.velocity[2]
                if self.in_air:
                    if self.no_vel_cmds:
                        current_timestamp = time.time()
                        dt = current_timestamp - self.prev_timestamp
                        self.prev_timestamp = current_timestamp
                        # Use PID controllers for velx and vely
                        pid_output_velx = self.calculate_pid_output(velx, self.pid_velx, 'velx',dt)
                        pid_output_vely = self.calculate_pid_output(vely, self.pid_vely, 'vely',dt)
                        pid_output_velz = self.calculate_pid_output(velz, self.pid_velz, 'velz',dt)
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

                        self.send_ned_velocity_drone(pid_output_velx, pid_output_vely, pid_output_velz)
                        time.sleep(0.2)
                if not self.in_air:
                    break

                
            except Exception as e:
                log("Poshold_Guided Error: {}".format(e))

    def calculate_pid_output(self, current_value, pid_params, axis, dt):
        # Proportional term
        error = 0.0 - current_value
        proportional = pid_params['P'] * error
        
        # Integral term
        if axis == 'velx':
            self.integral_velx += error * dt  # Accumulate error over time
            integral = pid_params['I'] * self.integral_velx
        elif axis == 'vely':
            self.integral_vely += error * dt
            integral = pid_params['I'] * self.integral_vely
        elif axis == 'velz':
            self.integral_velz += error * dt
            integral = pid_params['I'] * self.integral_velz
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
        threading.Thread(target=self.is_wifi_connected).start()

        while self.name != "STOP":
            try:
                self.altitude = self.vehicle.location.global_relative_frame.alt
                self.battery = self.vehicle.battery.voltage
                self.mode = self.vehicle.mode
                if self.wifi_status:
                    log('sec {} PosAlt: {}m \n      Current altitude : {}m\n      Current Battery {}V\n      Alt Difference {}\n      Wifi Status {}\n{}'.format(self.name,self.posalt, self.altitude, self.battery, self.altitude - self.posalt, str(self.wifi_status), str(self.mode)))
                    coordlat = str(self.vehicle.location.global_relative_frame.lat)
                    coordlon = str(self.vehicle.location.global_relative_frame.lon)
                    log("lat {} {}".format(self.name,coordlat))
                    log("lon {} {}".format(self.name,coordlon))
                if self.in_air:
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
                mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,  # frame
                0b0000111111000111,  # type_mask (only speeds enabled)
                0, 0, 0,  # x, y, z positions (not used)
                velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
                0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
                0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

            self.vehicle.send_mavlink(msg)
            log("Drone Velocity : {}, {}, {}".format(velocity_x,velocity_y,velocity_z))

        except Exception as e:
            log(f"Error sending velocity commands: {e}")

    def send_ned_position_drone(self, disx, disy, disz):
        try:
            disx = float(disx)
            disy = float(disy)
            disz = float(disz)

            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,  # time_boot_ms (not used)
                0, 0,  # target system, target component
                mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,  # frame
                0b0000111111111000,  # type_mask (only positions enabled)
                disx, disy, disz,  # x, y, z positions (not used)
                0, 0, 0,  # x, y, z velocity in m/s
                0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
                0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

            self.vehicle.send_mavlink(msg)
            log("Drone heading to : {}m Forward, {}m Left, {}m Upward".format(disx,disy,disz))

        except Exception as e:
            log(f"Error sending position commands: {e}")

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

    def send_pos(self,x,y,z,duration=5):
        self.no_vel_cmds = False
        self.vehicle.groundspeed = 0.5
        self.send_ned_position_drone(x,y,z)
        time.sleep(duration)

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
                30,  # param 2, yaw speed deg/s
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

    def circle(self, radius=3, start_theta=0, velocity=0.5):
        T = 2 * math.pi * radius / velocity
        dt = 0.5

        for t in range(int(T / dt)):
            theta = start_theta + (2 * math.pi * t * dt / T)
            north_velocity = velocity * math.cos(theta)
            east_velocity = velocity * math.sin(theta)

            self.send_ned_velocity_drone(north_velocity, east_velocity, 0)
            time.sleep(dt)
            if not self.in_air:
                break


    def move_square_to_circle(self, start_index, radius=2, velocity=0.5):
        try:
            side_length = 2 * radius  # Assuming the square side length is equal to the diameter of the circle
            half_side = side_length / 2

            # Square positions
            square_positions = [
                (0, 0),          # A
                (0, side_length),  # B
                (side_length, side_length),  # C
                (side_length, 0)   # D
            ]

            current_index = start_index % 4
            next_index = (start_index + 1) % 4

            current_pos = square_positions[current_index]
            target_pos = square_positions[next_index]

            target_x, target_y = self.calculate_circle_position(current_pos[0], current_pos[1], target_pos[0], target_pos[1], half_side, radius, velocity)
            self.send_ned_velocity(target_x, target_y, 0, duration=2)  # Adjust duration based on the drone's speed

        except Exception as e:
            log(f"Error moving drones from square to circle: {e}")

    def move_pentagon_to_circle(self, start_index, radius=2, velocity=0.5):
        try:
            # Calculate pentagon side length based on radius and golden ratio
            side_length = 2 * radius * math.sin(math.pi / 5)
            half_side = side_length / 2

            # Pentagon positions, centered at (half_side, half_side)
            pentagon_positions = [
                (0, side_length),   # A
                (side_length * math.cos(math.pi / 5), half_side),  # B
                (side_length * math.cos(math.pi / 5), -half_side),  # C
                (0, -side_length),  # D
                (-side_length * math.cos(math.pi / 5), -half_side)  # E
            ]

            current_index = start_index % 5
            next_index = (start_index + 1) % 5

            current_pos = pentagon_positions[current_index]
            target_pos = pentagon_positions[next_index]

            target_x, target_y = self.calculate_circle_position(
                current_pos[0] + half_side,  # Offset for centering
                current_pos[1] + half_side,  # Offset for centering
                target_pos[0] + half_side,  # Offset for centering
                target_pos[1] + half_side,  # Offset for centering
                half_side, radius, velocity
            )
            self.send_ned_velocity(target_x, target_y, 0, duration=2)  # Adjust duration as needed

        except Exception as e:
            log(f"Error moving drones from pentagon to circle: {e}")

    def calculate_circle_position(self, x_start, y_start, x_end, y_end, half_side, radius, velocity):
        theta_start = math.atan2(y_start - half_side, x_start - half_side)
        theta_end = math.atan2(y_end - half_side, x_end - half_side)
        
        # Use the average of start and end thetas for a smoother transition
        theta_avg = (theta_start + theta_end) / 2

        # Calculate circular path components based on polar coordinates
        velocity_x = velocity * math.cos(theta_avg)
        velocity_y = velocity * math.sin(theta_avg)

        return velocity_x, velocity_y



    # def servo(self,cmd):
    #     delay_period = 0.01
    #     close = 'close'
    #     open = 'open'
    #     try:
    #         if cmd == 'close' or cmd == close:
    #             for pulse in range(50, 250, 1):
    #                 wiringpi.pwmWrite(18, pulse)
    #                 time.sleep(delay_period)
    #         if cmd == 'open' or cmd == open:
    #             for pulse in range(250, 50, -1):
    #                 wiringpi.pwmWrite(18, pulse)
    #                 time.sleep(delay_period)
    #         log('setting servo to {}'.format(cmd))
    #     except Exception as e:
    #         log(f"Error during servo command: {e}")

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

    def moder(self, cmd):
        mode_name = str(cmd)
        self.vehicle.mode = VehicleMode(mode_name)
        log("{} Mode changed to {}".format(self.name,cmd))

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
pc = '172.217.28.100'

import random

def send(host, immediate_command_str):
    global connected_hosts
    global clients

    try:
        if host not in connected_hosts:
            connect_and_register_socket(host)

        immediate_command_str = str(immediate_command_str)
        clients[host].send_string(immediate_command_str, zmq.NOBLOCK)  # Non-blocking send
        log("Command sent successfully to {}".format(host))

    except zmq.error.Again:  # Handle non-blocking send errors
        poller.register(clients[host], zmq.POLLOUT)  # Wait for socket readiness
        socks = dict(poller.poll(1000))
        if clients[host] in socks and socks[clients[host]] == zmq.POLLOUT:
            clients[host].send_string(immediate_command_str)  # Retry sending
        else:
            log(f"Socket not ready for {host}, reconnecting...")
            reconnect_socket(host)

    except zmq.error.ZMQError as e:
        log(f"PC Host {host}: {e}")
        reconnect_socket(host)  # Attempt reconnection

def connect_and_register_socket(host):
    socket = context.socket(zmq.PUSH)
    socket.setsockopt(zmq.SNDHWM, 1000)  # Allow up to 1000 queued messages
    socket.connect(f"tcp://{host}:12345")
    # poller.register(socket, zmq.POLLOUT)  # Register for write events
    clients[host] = socket
    connected_hosts.add(host)
    log("Clients: {}".format(clients))

def reconnect_socket(host):
    socket = clients[host]
    socket.close()
    socket = context.socket(zmq.PUSH)
    socket.connect(f"tcp://{host}:12345")
    # poller.register(socket, zmq.POLLOUT)
    clients[host] = socket
    socks = dict(poller.poll(1000))  # Wait for write events with timeout
    # return socket in socks and socks[socket] == zmq.POLLOUT

def camera_start():

    socket = context.socket(zmq.STREAM)  # Use STREAM socket
    socket.bind("tcp://*:8000")  # Bind to port 8000

    camera = None

    def handle_client():
        global camera

    #     while True:
    #         try:
    #             # Start the camera if not already running
    #             if not camera:
    #                 camera = picamera.PiCamera()
    #                 camera.resolution = (640, 480)  # Adjust as needed
    #                 camera.framerate = 30  # Adjust as needed
    #                 time.sleep(2)  # Camera warmup

    #             stream = io.BytesIO()
    #             for _ in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
    #                 stream.seek(0)
    #                 image_data = stream.read()

    #                 # Send image size and data
    #                 socket.send(struct.pack('<L', len(image_data)), flags=zmq.SNDMORE)
    #                 socket.send(image_data)

    #                 stream.seek(0)
    #                 stream.truncate()

    #         except Exception as e:
    #             print("Error:", e)

    #         finally:
    #             # Close client connection and camera if needed
    #             if socket:
    #                 socket.close()
    #             if camera:
    #                 camera.close()
    #                 camera = None

    # client_thread = threading.Thread(target=handle_client)
    # client_thread.start()
    # client_thread.isDaemon(True)


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
         
def chat(string):
    try:
        print(string)

    except Exception as e:
        log(f"Error in chat function: {e}")


context = zmq.Context()
dealer_socket = context.socket(zmq.DEALER)  # Create a single DEALER socket
dealer_socket.connect(f"tcp://{pc}:5556")  # Connect to the server

def log(immediate_command_str):
    try:
        immediate_command_str = str(immediate_command_str)
        dealer_socket.send_multipart([immediate_command_str.encode()])
        if not wifi_status:
            print(immediate_command_str)

    except zmq.ZMQError as e:
        print("Error sending message: %s", e)  # Log error
        # Retry queue
        

def file_server():
    try:
        context = zmq.Context()
        socket = context.socket(zmq.REP)

        # Change 'your_port' to the actual port you want to use
        socket.bind(f"tcp://{MCU_host}:5577")
        print("File_recieve server Started in MCU!")

        print("Waiting for file_name")
        file_name = socket.recv_string()
        socket.send_string("Completed")
        print("File Name Recieved")
        print("waiting for data")
        file_data = socket.recv()

        
        with open(f"{file_name}.txt", 'wb') as file:
            file.write(file_data)

        # Send a response back to the client (EDIT application)
        socket.send_string("File received successfully")
        time.sleep(1)
        socket.close()
        context.term()
        missions[file_name] = file
        log("MCU has {} Missons".format(str(missions)))
    except Exception as e:
        log("File_Server Error in MCU : {}".format(e))
