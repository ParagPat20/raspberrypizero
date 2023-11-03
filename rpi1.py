from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import socket
import threading
import geopy
import geopy.distance
from geopy.distance import great_circle
import io
import picamera
import struct

global Drone_ID
global drone1
global drone2

############################################################################################

class Drone:
    def __init__(self, connection_string, baudrate=None):
        self.vehicle = connect(connection_string, baud=baudrate)

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z):
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

    def takeoff(self):
        global stop
        print("Taking off!")
        self.vehicle.simple_takeoff(2)
        start_time = time.time()
        TIMEOUT_SECONDS = 15
        while True:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            if current_altitude is not None:
                print(" Altitude: ", current_altitude)
                if current_altitude >= 1 * 0.9:
                    print("Reached target altitude")
                    break
            else:
                print("Waiting for altitude information...")
            if time.time() - start_time > TIMEOUT_SECONDS:
                break
            time.sleep(1)

    def arm(self, mode='GUIDED'):
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

    def yaw(self,heading):
    # create the CONDITION_YAW command using command_long_encode()
        current_heading = self.vehicle.heading
        print("Current Heading : ", current_heading)
        if current_heading >= 180:
            rotation = 1
        else:
            rotation = -1
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
############################################################################################

def control_servo(servo, angle):
    if -1 <= angle <= 1:
        servo.value = angle
        return True
    else:
        print("Invalid angle. Angle should be between -1 and 1.")
        return False
    
############################################################################################

def camera_stream_server(host):
    def handle_client(client_socket):
        connection = client_socket.makefile('wb')

        try:
            with picamera.PiCamera() as camera:
                camera.resolution = (640, 480)  # Adjust resolution as needed
                camera.framerate = 30  # Adjust frame rate as needed

                # Start capturing and sending the video feed
                time.sleep(2)  # Give the camera some time to warm up
                stream = io.BytesIO()
                for _ in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
                    stream.seek(0)
                    image_data = stream.read()

                    # Send the image size to the client
                    connection.write(struct.pack('<L', len(image_data)))
                    connection.flush()

                    # Send the image data to the client
                    connection.write(image_data)
                    stream.seek(0)
                    stream.truncate()
        except Exception as e:
            print("Error: ", e)

        finally:
            connection.close()
            client_socket.close()

    # Create a socket server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, 8000))
    server_socket.listen(0)

    print("Server is listening on {}:{}".format(host, 8000))

    while True:
        client_socket, _ = server_socket.accept()
        client_thread = threading.Thread(target=handle_client, args=(client_socket,))
        client_thread.start()


############################################################################################

############################################################################################

def ServerSendGPS(drone,local_host,port):
    gps_socket = socket.socket()
    gps_socket.bind((local_host, port))
    gps_socket.listen(2)

    print('{} - SERVER_Send_GPS() is started!'.format(time.ctime()))

    while True:
        try:
            client_connection, client_address = gps_socket.accept()
            print('\n{} - Received GPS Request from {}.'.format(time.ctime(),client_address))

            gps_data = gps(drone)
            client_connection.send(gps_data.encode())

        except KeyboardInterrupt:
            # Handle KeyboardInterrupt to gracefully exit the loop
            break

        except Exception as e:
            print(f"Error: {e}")
            time.sleep(1)
        finally:
            if client_connection:
                client_connection.close()
            time.sleep(1)
    gps_socket.close()

def gps(drone):
    lat = '{:.7f}'.format(drone.vehicle.location.global_relative_frame.lat) if hasattr(drone.vehicle, 'location') else '0'
    lon = '{:.7f}'.format(drone.vehicle.location.global_relative_frame.lon) if hasattr(drone.vehicle, 'location') else '0'
    alt = str(drone.vehicle.location.global_relative_frame.alt) if hasattr(drone.vehicle, 'location') else '0'

    gps_str = ','.join([lat, lon, alt])

    return gps_str

############################################################################################

def ClientRequestGPS(remote_host,port):
    # Create a socket object
    client_socket = socket.socket()
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        client_socket.connect((remote_host,port))
    except socket.error as error_msg:
        print('{} - Caught exception : {}'.format(time.ctime(), error_msg))
        print('{} - CLIENT_request_gps({}) is not executed!'.format(time.ctime(), remote_host))
        return None, None, None
    gps_msg_str = client_socket.recv(1024).decode()
    print('Recieved GPS Params :', gps_msg_str)
    # Return lat, lon, and alt
    lat, lon, alt = gps_msg_str.split(',')
    return float(lat), float(lon), float(alt)

def SERVER_CTRL(local_host):
    global ctrl_port
    global Drone_ID, drone1, drone2
    control_socket = socket.socket()
    control_socket.bind((local_host, ctrl_port))
    control_socket.listen(1)

    print('{} - SERVER_receive_control_commands() is started!'.format(time.ctime()))

    while True:
        if Drone_ID == drone1 or Drone_ID == drone2:
            try:
                client_connection, client_address = control_socket.accept()
                print('\n{} - Received control command from {}.'.format(time.ctime(), client_address))

                control_command_str = client_connection.recv(1024).decode()  # Receive and decode the command

                print('{} - Control command is: {}'.format(time.ctime(), control_command_str))
                
                try:
                    d, x, y, z = control_command_str.split(',') # Split
                    x,y,z = float(x),float(y),float(z)
                    print(d)
                    if d == "MCU":
                        CTRL(MCU,x,y,z)
                    if d == "CD1":
                        CTRL(CD1,x,y,z)
                    if d == "CD2":
                        CLIENT_send_immediate_command(CD2_host,'CTRL(drone1,'+str(x)+','+str(y)+','+str(z)+')')
                        print("Sent Command")
                    if d == "CD3":
                        CLIENT_send_immediate_command(CD2_host,'CTRL(drone2,'+str(x)+','+str(y)+','+str(z)+')')                        
                        print("Sent Command2")
                    # if d == 'CD4':
                    #     C(CD4_host,drone1,x,y,z)
                    # if d == 'CD5':
                    #     C(CD4_host,drone2,x,y,z)

                except ValueError:
                    print("Invalid control command format. Expected 'd,x,y,z'")
            except KeyboardInterrupt:
                # Handle KeyboardInterrupt to gracefully exit the loop
                break
            except Exception as e:
                # Handle other exceptions, e.g., if the client disconnects unexpectedly
                print(f"Error: {e}")
                time.sleep(1)
            finally:
                if client_connection:
                    client_connection.close()
        else:
            print("Not Controlling any drone herer")
            time.sleep(1)

############################################################################################
def SERVER_receive_and_execute_immediate_command(local_host):
    global cmd_port
    global status_waitForCommand
    # Create a socket object
    msg_socket = socket.socket()
    msg_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Bind to the port
    msg_socket.bind((local_host, cmd_port))
    msg_socket.listen(5)                 # Now wait for client connection.
    print('{} - SERVER_receive_and_execute_immediate_command() is started!'.format(time.ctime()))
    while True:
        try:
            client_connection, client_address = msg_socket.accept() # Establish connection with client.
            print('\n{} - Received immediate command from {}.'.format(time.ctime(), client_address))
            immediate_command_str = client_connection.recv(1024).decode()
            print('{} - Immediate command is: {}'.format(time.ctime(), immediate_command_str))
            
            if status_waitForCommand == True:
                exec(immediate_command_str)
                status_waitForCommand = True
                print('{} - Immediate command \'{}\' is finished!'.format(time.ctime(), immediate_command_str))
            elif immediate_command_str == 'status(True)':
                exec(immediate_command_str)
            else:
                print('{} - Omit immediate command \'{}\', because status_waitForCommand is False!'.format(time.ctime(), immediate_command_str))
        except KeyboardInterrupt:
            # Handle KeyboardInterrupt to gracefully exit the loop
            break
        except Exception as e:
            # Handle other exceptions, e.g., if the client disconnects unexpectedly
            print(f"Error: {e}")
            time.sleep(1)
        finally:
            if client_connection:
                client_connection.close()

def CLIENT_send_immediate_command(remote_host, immediate_command_str):
    global cmd_port
    # Create a socket object
    client_socket = socket.socket()
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        client_socket.connect((remote_host, cmd_port))
    except socket.error as error_msg:
        print('{} - Caught exception : {}'.format(time.ctime(), error_msg))
        print('{} - CLIENT_send_immediate_command({}, {}) is not executed!'.format(time.ctime(), remote_host, immediate_command_str))
        return
    client_socket.send(immediate_command_str.encode())

############################################################################################

def wait_for_follower_ready(follower_host_tuple):
    all_follower_status = []
    for follower_host in follower_host_tuple:
        iter_follower_status = CLIENT_request_status(follower_host)
        all_follower_status.append(iter_follower_status)
    while not all(all_follower_status): # If there is anyone who is False.
        for i in range(len(follower_host_tuple)):
            if not all_follower_status[i]:
                print('{} - Host {} is not ready.'.format(time.ctime(), follower_host_tuple[i]))
        print('{} - Wait for 1 second.'.format(time.ctime()))
        time.sleep(1)
        # Reset status.
        all_follower_status = []
        # Check all followers' status again.
        for follower_host in follower_host_tuple:
            iter_follower_status = CLIENT_request_status(follower_host)
            all_follower_status.append(iter_follower_status)

def SERVER_send_status(local_host):
    global status_waitForCommand
    global st_port
    # Create a socket object
    msg_socket = socket.socket()
    msg_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Bind to the port
    msg_socket.bind((local_host, st_port))
    msg_socket.listen(5)                 # Now wait for client connection.
    print('{} - SERVER_send_status() is started!'.format(time.ctime()))
    while True:
        try:
            # msg_socket.accept() will block while loop until the connection with client is established.
            client_connection, client_address = msg_socket.accept() # Establish connection with client.
            print('{} - Received follower status request from {}.'.format(time.ctime(), client_address))
            # Send message to client.
            str_status_waitForCommand = str(int(status_waitForCommand))
            client_connection.send(str_status_waitForCommand)
            # Socket is destroyed when message has been sent.
        except KeyboardInterrupt:
            # Handle KeyboardInterrupt to gracefully exit the loop
            break
        except Exception as e:
            # Handle other exceptions, e.g., if the client disconnects unexpectedly
            print(f"Error: {e}")
            time.sleep(1)
        finally:
            if client_connection:
                client_connection.close()

def CLIENT_request_status(remote_host):
    global st_port
    # Create a socket object
    client_socket = socket.socket()
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        client_socket.connect((remote_host, st_port))
    except socket.error as error_msg:
        print('{} - Caught exception : {}'.format(time.ctime(), error_msg))
        print('{} - CLIENT_request_status({}) is not executed!'.format(time.ctime(), remote_host))
        return False
    status_msg_str = client_socket.recv(1024)
    return bool(int(status_msg_str))

############################################################################################

def goto(drone, lat, lon, alt, groundspeed = 1):
    print('{} - Calling goto_gps_location_relative(lat={}, lon={}, alt={}, groundspeed={}).'.format(time.ctime(), lat, lon, alt, groundspeed))
    destination = LocationGlobalRelative(lat, lon, alt)
    current_lat = drone.vehicle.location.global_relative_frame.lat
    current_lon = drone.vehicle.location.global_relative_frame.lon
    current_alt = drone.vehicle.location.global_relative_frame.alt

    while ((distance_between_two_gps_coord((current_lat,current_lon), (lat,lon)) >0.6) or (abs(current_alt-alt)>0.3)):
        # Execute fly command.
        drone.vehicle.simple_goto(destination,groundspeed=groundspeed)
        # wait for one second.
        time.sleep(0.5)
        current_lat = drone.vehicle.location.global_relative_frame.lat
        current_lon = drone.vehicle.location.global_relative_frame.lon
        current_alt = drone.vehicle.location.global_relative_frame.alt
        print('{} - Horizontal distance to destination: {} m.'.format(time.ctime(), distance_between_two_gps_coord((current_lat,current_lon), (lat,lon))))
        print('{} - Perpendicular distance to destination: {} m.'.format(time.ctime(), current_alt-alt))
        if drone.vehicle.mode == VehicleMode('LAND'):
            break

def new_coords(original_gps_coord, displacement, rotation_degree):
    vincentyDistance = geopy.distance.distance(meters = displacement)
    original_point = geopy.Point(original_gps_coord[0], original_gps_coord[1])
    new_gps_coord = vincentyDistance.destination(point=original_point, bearing=rotation_degree)
    new_gps_lat = new_gps_coord.latitude
    new_gps_lon = new_gps_coord.longitude
    # If convert float to decimal, round will be accurate, but will take 50% more time. Not necessary.
    #new_gps_lat = decimal.Decimal(new_gps_lat)
    #new_gps_lon = decimal.Decimal(new_gps_lon)
    return (round(new_gps_lat, 7), round(new_gps_lon, 7))

def distance_between_two_gps_coord(point1, point2):
    distance = great_circle(point1, point2).meters
    return distance

############################################################################################

def start_drone_server_services(drone, local_host, port):
    threading.Thread(target=ServerSendGPS, args=(drone, local_host, port,)).start()
def start_server(local_host):
        threading.Thread(target=SERVER_receive_and_execute_immediate_command, args=(local_host,)).start()
        threading.Thread(target=SERVER_send_status, args=(local_host,)).start()
        threading.Thread(target=SERVER_CTRL, args=(local_host,)).start()
        threading.Thread(target=camera_stream_server, args=(local_host,)).start()
############################################################################################

def ARM(drone):
    threading.Thread(target=drone.arm, args=('GUIDED',)).start()

def TAKEOFF(drone):
    threading.Thread(target=drone.takeoff).start()
    print(drone, "takeoff")

def LAND(drone):
    threading.Thread(target=drone.land).start()

def MODE(drone,mode):
    threading.Thread(target=set_mode, args= (drone,mode,)).start()

def set_mode(drone,mode):
    drone.vehicle.mode = VehicleMode(mode)

def YAW(drone, heading):
    threading.Thread(target=drone.yaw, args=(heading,)).start()

def LINEON(dis,alt):
    threading.Thread(target=LINE,args=(dis, alt)).start()

def SQUAREON(dis,alt):
    threading.Thread(target=SQUARE,args=(dis,alt)).start()

def TRION(dis,alt):
    threading.Thread(target=TRI, args=(dis,alt,)).start()

def ZIGZAGON(dis,alt):
    threading.Thread(target=ZIGZAG,args=(dis,alt)).start()

def CTRL(drone,x,y,z):
    drone.send_ned_velocity(x,y,z)
    time.sleep(0.3)
    drone.send_ned_velocity(0,0,0)

def D(drone):
    global Drone_ID
    Drone_ID = drone

def POSHOLDALL():
    global drone1, drone2
    MODE(drone1,'POSHOLD')
    MODE(drone2,'POSHOLD')

def POSHOLD(drone):
    MODE(drone,'POSHOLD')

def land_all():
    global drone1, drone2
    LAND(drone1)
    LAND(drone2)

def status(cmd):
    global status_waitForCommand
    status_waitForCommand = cmd

def in_line_done():
    print("Formation Completed Successfully! Recieved command from CD4 Host!")
    global in_line
    in_line = True

def in_square_done():
    print("SQUARE Completed Successfully! Recieved command from CD4 Host!")

def in_tri_done():
    print("TRIANGLES Completed Successfully! Recieved command from CD4 Host!")

def in_zigzag_done():
    print("ZIGZG Completed Successfully! Recieved command from CD4 Host!")

def in_circle_done():
    print("Circle Completed Successfully! Recieved command from CD4 Host!")

def ARMALL():
    print("Arming All Drones")
    ARM(drone1)
    ARM(drone2)
    CLIENT_send_immediate_command(CD2_host,'ARM(drone1)')
    CLIENT_send_immediate_command(CD2_host,'ARM(drone2)')
    # CLIENT_send_immediate_command(CD4_host,'ARM(drone1)')
    # CLIENT_send_immediate_command(CD4_host,'ARM(drone2)')
def LANDALL():
    print("LANDING All Drones")
    LAND(drone1)
    LAND(drone2)
    CLIENT_send_immediate_command(CD2_host,'LAND(drone1)')
    CLIENT_send_immediate_command(CD2_host,'LAND(drone2)')
    # CLIENT_send_immediate_command(CD4_host,'LAND(drone1)')
    # CLIENT_send_immediate_command(CD4_host,'LAND(drone2)')
def TAKEOFFALL():
    print("Taking off All Drones")
    TAKEOFF(drone1)
    TAKEOFF(drone2)
    CLIENT_send_immediate_command(CD2_host,'TAKEOFF(drone1)')
    CLIENT_send_immediate_command(CD2_host,'TAKEOFF(drone2)')
    # CLIENT_send_immediate_command(CD4_host,'TAKEOFF(drone1)')
    # CLIENT_send_immediate_command(CD4_host,'TAKEOFF(drone2)')


def set_servo(ang):
    global angle
    angle = ang

############################################################################################

local_host = '192.168.12.122' # change these
cmd_port = 12345
ctrl_port = 54321
st_port = 60001
status_waitForCommand = True
in_line = False

MCU_host = "192.168.12.122" # change these
CD2_host = "192.168.12.44" # change these
CD4_host = "192.168.149.103" # change these

MCU = Drone('/dev/serial0',baudrate=115200)
print("MCU connected")
CD1 = Drone('0.0.0.0:14552')
print("CD1 Connected")

drone1 = MCU
drone2 = CD1
Drone_ID = MCU


def cu_lo(drone):
    point = drone.vehicle.location.global_relative_frame
    return point

def LINE(dis = 2, alt = 2):
    pointA = cu_lo(MCU)
    cdis = 0
    A = (pointA.lat, pointA.lon)

    MCU.arm()
    print("MCU Reached and Fixed on its Position")
    YAW(MCU,0)
    POSHOLD(MCU)

    cdis = dis
    CD1.arm()
    B = new_coords(A,cdis,0)
    goto(CD1,B[0],B[1],alt,0.7)
    print("CD1 Reached and Fixed on its Position")
    YAW(CD1,0)
    POSHOLD(CD1)

    CLIENT_send_immediate_command(CD2_host, 'LINEON('+str(dis)+','+str(alt)+')')

def SQUARE(dis = 2, alt = 2):
    if in_line == True:
        pointA = cu_lo(MCU)
        cdis = dis * 0
        A = (pointA.lat, pointA.lon)
        MCU.arm()
        goto(MCU,A[0],A[1],alt,0.7)
        print("MCU Reached and Fixed on its Position")
        YAW(MCU,0)
        POSHOLD(MCU)

        cdis = dis * 1
        CD1.arm()
        B = new_coords(A,cdis,90)
        goto(CD1,B[0],B[1],alt,0.7)
        print("CD1 Reached and Fixed on its Position")
        YAW(CD1,0)
        POSHOLD(CD1)
        CLIENT_send_immediate_command(CD2_host, 'SQUAREON('+str(dis)+','+str(alt)+')')
        in_line = False
    else:
        print("Drones are not in Line")
        time.sleep(1)
        print("Run LINE()")

def ZIGZAG(dis = 2, alt = 2):
    if in_line == True:
        pointA = cu_lo(CD1)
        cdis = dis*0
        A=(pointA.lat,pointA.lon)
        POSHOLD(MCU)
        CD1.arm()
        cdis = dis*1
        B = new_coords(A,cdis,90)
        goto(CD1,B[0],B[1],alt,0.7)
        print("CD1 Reached and Fixed on its Position")
        YAW(CD1,0)
        POSHOLD(CD1)

        CLIENT_send_immediate_command(CD2_host, 'ZIGZAGON('+str(dis)+','+str(alt)+')')
        in_line = False
    else:
        print("Drones are not in Line")
        time.sleep(1)
        print("Run LINE()")

def TRI(dis = 2, alt = 2):
    if in_line == True:
        pointA = cu_lo(MCU)
        cdis = dis*0
        A=(pointA.lat,pointA.lon)
        POSHOLD(MCU)

        cdis = dis*1
        CD1.arm()
        B = new_coords(A,cdis,45)
        goto(CD1,B[0],B[1],alt,0.7)
        print("CD1 Reached and Fixed on its Position")
        YAW(CD1,0)
        POSHOLD(CD1)

        CLIENT_send_immediate_command(CD2_host, 'TRION('+str(dis)+','+str(alt)+')')
        in_line = False
    else:
        print("Drones are not in Line")
        time.sleep(1)
        print("Run LINE()")

def CIRCLE(dis = 1, alt = 2):
    if in_line == True:
        pointA = ClientRequestGPS(CD2_host,60004)
        lat, lon, alt = pointA
        A = (lat,lon)

        cdis = dis*2
        B = new_coords(A,cdis,180)
        goto(MCU,B[0],B[1],alt,0.7)
        print("MCU Reached and Fixed on its Position")
        YAW(MCU,0)

        C = new_coords(A,cdis,108)
        goto(CD1,C[0],C[1],alt,0.7)
        print("CD1 Reached and Fixed on its Position")
        YAW(CD1,0)

        CLIENT_send_immediate_command(CD2_host, 'CIRCLE('+str(dis)+','+str(alt)+')')
    else:
        print("Drones are not in Line")
        time.sleep(1)
        print("Run LINE()")

def FRAME():
    if in_line == True:
        A = cu_lo(MCU)
        goto(MCU, A.lat, A.lon, 1, 0.3)
        B = cu_lo(CD1)
        goto(CD1, B.lat, B.lon, 2, 0.3)
        YAW(MCU,0)
        YAW(CD1,0)
        CLIENT_send_immediate_command(CD2_host,'FRAME()')
    else:
        print("Drones are not in Line")
        time.sleep(1)
        print("Run LINE()")
    

start_server(local_host)
start_drone_server_services(MCU, local_host,60002)
start_drone_server_services(CD1, local_host,60003)
