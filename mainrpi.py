from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import socket
import threading
import geopy
import geopy.distance
from geopy.distance import great_circle

local_host = '0.0.0.0'
remote_host = '192.168.155.122'
mode_port = 12345
ctrl_port = 60003
status_port = [60002,60004]
gps_port = [60015,60016]
gps_server_port = [60010,60011]

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
        self.vehicle.simple_takeoff(1)
        start_time = time.time()
        TIMEOUT_SECONDS = 10
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

        while not self.vehicle.armed:
            print("Waiting for Arming")
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


MCU = Drone('/dev/serial0',baudrate=115200)
print("MCU connected")
CD1 = Drone('0.0.0.0:14552')
print("CD1 Connected")

# MCU = Drone('tcp:127.0.0.1:5762')
# print("MCU connected")
# CD1 = Drone('tcp:127.0.0.1:5772')
# print("CD1 Connected")

Drone_ID = MCU

def ServerSendStatus(drone, local_host, status_port):
    status_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    status_socket.bind((local_host, status_port))
    status_socket.listen(1)

    print('{} - ServerSendStatus is started!'.format(time.ctime()))

    while True:
        try:
            client_connection, client_address = status_socket.accept()

            status_data = status(drone)  # Call your status function here
            client_connection.send(status_data.encode())  # Send the string

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
            time.sleep(1)

    status_socket.close()

def status(drone):
    vehicle = drone.vehicle

    if vehicle is None:
        return "Not connected"

    b = str(vehicle.battery.voltage) if vehicle.battery is not None else '0'
    gs = str(vehicle.groundspeed) if hasattr(vehicle, 'groundspeed') else '0'
    md = str(vehicle.mode) if hasattr(vehicle, 'mode') else 'UNKNOWN'
    vx = str(vehicle.velocity[0]) if hasattr(vehicle, 'velocity') and len(vehicle.velocity) > 0 else '0'
    vy = str(vehicle.velocity[1]) if hasattr(vehicle, 'velocity') and len(vehicle.velocity) > 1 else '0'
    vz = str(vehicle.velocity[2]) if hasattr(vehicle, 'velocity') and len(vehicle.velocity) > 2 else '0'
    gps = str(vehicle.gps_0.fix_type) if hasattr(vehicle, 'gps_0') and hasattr(vehicle.gps_0, 'fix_type') else '0'
    lat = '{:.7f}'.format(vehicle.location.global_relative_frame.lat) if hasattr(vehicle, 'location') else '0'
    lon = '{:.7f}'.format(vehicle.location.global_relative_frame.lon) if hasattr(vehicle, 'location') else '0'
    alt = str(vehicle.location.global_relative_frame.alt) if hasattr(vehicle, 'location') else '0'
    armed = str(vehicle.armed) if hasattr(vehicle, 'armed') else 'False'

    status_str = ','.join([b, gs, md, vx, vy, vz, gps, lat, lon, alt, armed])

    return status_str

def reconnectdrone(drone,connection,baud=None):
    drone.exit()
    time.sleep(1)
    drone = Drone(connection_string=connection,baudrate=baud)
    print("Drone Reconnected Successfully!")


def ServerRecvCmd(local_host):
    global mode_port
    global Drone_ID
    cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cmd_socket.bind((local_host, mode_port))
    cmd_socket.listen(1)

    print('{} - SERVER_receive_and_execute_immediate_command() is started!'.format(time.ctime()))

    while True:
        try:
            client_connection, client_address = cmd_socket.accept()
            print('\n{} - Received immediate command from {}.'.format(time.ctime(), client_address))

            immediate_command_str = client_connection.recv(1024).decode()  # Receive and decode the command

            print('{} - Immediate command is: {}'.format(time.ctime(), immediate_command_str))
            
            if immediate_command_str == 'MCU':
                Drone_ID = MCU
                MCU.land()
                print("Reconnecting MCU Drone................................................................")
                time.sleep(1)
                threading.Thread(target=reconnectdrone, args=(MCU,'/dev/serial0',115200,)).start()
            if immediate_command_str == 'CD1':
                Drone_ID = CD1
                CD1.land()
                print("Reconnecting MCU Drone................................................................")
                time.sleep(1)
                threading.Thread(target=reconnectdrone, args=(CD1,'0.0.0.0:14552',)).start()
            if immediate_command_str == 'ARM':
                drone_arm(MCU)
                drone_arm(CD1)
            if immediate_command_str == 'land_all':
                drone_land(MCU)
                drone_land(CD1)
            if immediate_command_str == 'TakeOff':
                Drone_ID.takeoff()
            if immediate_command_str == 'takeoff':
                drone_takeoff(MCU)
                drone_takeoff(CD1)
            if immediate_command_str == 'POSHOLD':
                drone_mode(MCU, 'POSHOLD')
                drone_mode(CD1, 'POSHOLD')
            if immediate_command_str == 'line':
                threading.Thread(target=line, args=(2,3)).start()
            
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

def ServerRecvControl(local_host):
    global ctrl_port
    global Drone_ID
    control_socket = socket.socket()
    control_socket.bind((local_host, ctrl_port))
    control_socket.listen(1)

    print('{} - SERVER_receive_control_commands() is started!'.format(time.ctime()))

    while True:
        try:
            client_connection, client_address = control_socket.accept()
            print('\n{} - Received control command from {}.'.format(time.ctime(), client_address))

            control_command_str = client_connection.recv(1024).decode()  # Receive and decode the command

            print('{} - Control command is: {}'.format(time.ctime(), control_command_str))
            
            try:
                x, y, z = map(float, control_command_str.split(','))  # Split and convert to floats
                drone_vel_ctrl(MCU, x, y, z)
                drone_vel_ctrl(CD1, x, y, z)
            except ValueError:
                print("Invalid control command format. Expected 'x,y,z'")
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

def drone_ctrl(drone,x,y,z):
    threading.Thread(target=drone.position_target_local_ned, args=(x, y, z,)).start()

def drone_vel_ctrl(drone,x,y,z):
    threading.Thread(target=drone.send_ned_velocity, args=(x, y, z,)).start()

def drone_takeoff(drone):
    threading.Thread(target=drone.takeoff).start()
    print(drone, "takeoff")

def drone_arm(drone):
    threading.Thread(target=drone.arm, args=('GUIDED',)).start()

def drone_land(drone):
    threading.Thread(target=drone.land).start()

def set_mode(drone,mode):
    drone.vehicle.mode = VehicleMode(mode)

def drone_mode(drone,mode):
    threading.Thread(target=set_mode, args= (drone,mode,)).start()

def start_server_service(local_host):
    threading.Thread(target=ServerRecvCmd, args=(local_host,)).start()
    threading.Thread(target=ServerRecvControl, args=(local_host,)).start()
    print("Thread serverRecvMode and ServerRecvControl are started!")

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
    print('{} - After calling goto_gps_location_relative(), vehicle state is:'.format(time.ctime()))

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


def cu_lo(drone):
    point = drone.vehicle.location.global_relative_frame
    return point

def line(dis = 2, alt = 2):
    pointA = cu_lo(MCU)
    cdis = 0
    A = (pointA.lat, pointA.lon)
    cdis = cdis + dis
    B = new_coords(A,cdis,0)
    cdis = cdis + dis
    goto(CD1,B[0],B[1],alt,0.7)
    time.sleep(1)
    print("Line Completed")
    

def main():

    start_server_service(local_host)
    threading.Thread(target=ServerSendStatus, args=(MCU, local_host, status_port[0],)).start()
    print("MCU SendStatus Active")
    threading.Thread(target=ServerSendStatus, args=(CD1, local_host, status_port[1],)).start()
    print("CD1 SendStatus Active")
    threading.Thread(target=ServerSendGPS,args=(MCU,local_host,gps_server_port[0])).start()
    print("MCU GPS Server started")
    threading.Thread(target=ServerSendGPS,args=(CD1,local_host,gps_server_port[1])).start()
    print("CD1 GPS Server started")

time.sleep(2)
main()

