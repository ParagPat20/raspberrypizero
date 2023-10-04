from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import socket
import threading
import geopy
import geopy.distance
from geopy.distance import great_circle

local_host = '0.0.0.0'
remote_host = '192.168.155.101'
mode_port = 60001
ctrl_port = 60003
status_port = [60002,60004,60006,60008]
stop = 0

class Drone:
    def __init__(self, connection_string, baudrate=None):
        self.vehicle = connect(connection_string, baud=baudrate)

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z,duration):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        for x in range(0,duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1) 
    
    def position_target_local_ned(self, north, east, down):
        """
        Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
        location in the North, East, Down frame.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111111000, # type_mask (only positions enabled)
            north, east, down,
            0, 0, 0, # x, y, z velocixty in m/s  (not used)
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        time.sleep(5)

    def takeoff(self):
        global stop
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
            if stop == 1:
                break
            time.sleep(1)

    def arm(self, mode='GUIDED'):
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


MCU = Drone('/dev/serial0',baudrate=115200)
print("MCU connected")
CD1 = Drone('0.0.0.0:14550')
print("CD1 Connected")
CD2 = Drone('0.0.0.0:14552')
print("CD2 Connected")
CD3 = Drone('0.0.0.0:14553')
print("CD3 Connected")
# MCU = Drone('tcp:127.0.0.1:5762')
# print("MCU connected")
# CD1 = Drone('tcp:127.0.0.1:5772')
# print("CD1 Connected")
# CD2 = Drone('tcp:127.0.0.1:5782')
# print("CD2 Connected")
# CD3 = Drone('tcp:127.0.0.1:5792')
# print("CD3 Connected")
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

            client_connection.close()
        except KeyboardInterrupt:
            # Handle KeyboardInterrupt to gracefully exit the loop
            break
        except Exception as e:
            # Handle other exceptions, e.g., if the client disconnects unexpectedly
            print(f"Error: {e}")
            time.sleep(1)
        finally:
            time.sleep(1)

    status_socket.close()


def status(drone):
    b = str(drone.vehicle.battery.voltage) if drone.vehicle.battery.voltage is not None else '0'
    gs = str(drone.vehicle.groundspeed)
    md = str(drone.vehicle.mode)
    vx = str(drone.vehicle.velocity[0])
    vy = str(drone.vehicle.velocity[1])
    vz = str(drone.vehicle.velocity[2])
    gps = str(drone.vehicle.gps_0.fix_type) if drone.vehicle.gps_0.fix_type is not None else '0'
    lat = '{:.7f}'.format(drone.vehicle.location.global_relative_frame.lat)
    lon = '{:.7f}'.format(drone.vehicle.location.global_relative_frame.lon)
    alt = str(drone.vehicle.location.global_relative_frame.alt)
    armed = str(drone.vehicle.armed)

    status_str = ','.join([b, gs, md, vx,vy,vz, gps, lat, lon, alt, armed])

    return status_str

def ServerRecvCmd(local_host):
    global mode_port
    global Drone_ID
    cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cmd_socket.bind((local_host, mode_port))
    cmd_socket.listen(1)

    print('{} - SERVER_receive_and_execute_immediate_command() is started!'.format(time.ctime()))

    while True:
        client_connection, client_address = cmd_socket.accept()
        print('\n{} - Received immediate command from {}.'.format(time.ctime(), client_address))

        immediate_command_str = client_connection.recv(1024).decode()  # Receive and decode the command

        print('{} - Immediate command is: {}'.format(time.ctime(), immediate_command_str))
        if immediate_command_str == 'MCU':
            Drone_ID = MCU
        if immediate_command_str == 'CD1':
            Drone_ID = CD1
        if immediate_command_str == 'CD2':
            Drone_ID = CD2
        if immediate_command_str == 'CD3':
            Drone_ID = CD3
        if immediate_command_str == 'ARM':
            drone_arm(MCU)
            drone_arm(CD1)
            drone_arm(CD2)
            drone_arm(CD3)
        if immediate_command_str == 'LAND':
            Drone_ID.land()
        if immediate_command_str == 'land_all':
            drone_land(MCU)
            drone_land(CD1)
            drone_land(CD2)
            drone_land(CD3)
        if immediate_command_str == 'TakeOff':
            Drone_ID.takeoff()
        if immediate_command_str == 'takeoff':
            drone_takeoff(MCU)
            drone_takeoff(CD1)
            drone_takeoff(CD2)
            drone_takeoff(CD3)
        if immediate_command_str == 'POSHOLD':
            drone_mode(MCU,'POSHOLD')
            drone_mode(CD1,'POSHOLD')
            drone_mode(CD2,'POSHOLD')
            drone_mode(CD3,'POSHOLD')
        if immediate_command_str == 'square':
            print("Square")
            square()
        if immediate_command_str == 'line':
            print("Line")
            line()
        if immediate_command_str == 'tri':
            print("Triangle")
            tri()

        client_connection.close()

def ServerRecvControl(local_host):
    global ctrl_port
    global Drone_ID
    control_socket = socket.socket()
    control_socket.bind((local_host, ctrl_port))
    control_socket.listen(1)

    print('{} - SERVER_receive_control_commands() is started!'.format(time.ctime()))

    while True:
        client_connection, client_address = control_socket.accept()
        print('\n{} - Received control command from {}.'.format(time.ctime(), client_address))

        control_command_str = client_connection.recv(1024).decode()  # Receive and decode the command

        print('{} - Control command is: {}'.format(time.ctime(), control_command_str))
        
        try:
            x, y, z = map(float, control_command_str.split(','))  # Split and convert to floats
            Drone_ID.send_ned_velocity(x, y, z, 1)
        except ValueError:
            print("Invalid control command format. Expected 'x,y,z'")

        client_connection.close()

def drone_ctrl(drone,x,y,z):
    threading.Thread(target=drone.position_target_local_ned, args=(x, y, z,)).start()

def drone_vel_ctrl(drone,x,y,z,t):
    threading.Thread(target=drone.send_ned_velocity, args=(x, y, z, t,)).start()

def drone_takeoff(drone):
    threading.Thread(target=drone.takeoff).start()
    print(drone, "takeoff")

def drone_arm(drone):
    threading.Thread(target=drone.arm, args=('STABILIZE',)).start()

def drone_land(drone):
    threading.Thread(target=drone.land).start()

def set_mode(drone,mode):
    drone.vehicle.mode = VehicleMode(mode)

def drone_mode(drone,mode):
    threading.Thread(target=set_mode, args= (drone,mode,)).start

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
        drone.vehicle.simple_goto(destination)
        # wait for one second.
        time.sleep(0.5)
        current_lat = drone.vehicle.location.global_relative_frame.lat
        current_lon = drone.vehicle.location.global_relative_frame.lon
        current_alt = drone.vehicle.location.global_relative_frame.alt
        print('{} - Horizontal distance to destination: {} m.'.format(time.ctime(), distance_between_two_gps_coord((current_lat,current_lon), (lat,lon))))
        print('{} - Perpendicular distance to destination: {} m.'.format(time.ctime(), current_alt-alt))
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

def set_yaw(drone, yaw_inDegree):
    print('\n')
    print('{} - Calling function set_yaw(yaw_inDegree={}).'.format(time.ctime(), yaw_inDegree))
    # Do not pass True of False into msg, just in case the conversion is unpredictable.

    degreeToTurn = yaw_inDegree
    if degreeToTurn < 180:
        degreeToTurn = 360 - degreeToTurn
        estimatedTime = degreeToTurn/30.0 + 1 # Upon testing, the turning speed is 30 degree/second. Add one more second.
        print('{} - Absolute degree to turn is {} degree. Estimated time is {} seconds.'.format(time.ctime(), degreeToTurn, estimatedTime))
    else:
        is_relative = 0
        print('{} - The target degree is absolute degree[0~360](0=North, 90=East).'.format(time.ctime()))
        currentHeading = drone.vehicle.heading
        print('{} - Current heading is {} degree.'.format(time.ctime(), currentHeading))
        print('{} - Target heading is {} degree.'.format(time.ctime(), yaw_inDegree))
        degreeToTurn = abs(yaw_inDegree - drone.vehicle.heading)
        if degreeToTurn < 180:
            degreeToTurn = 360 - degreeToTurn
        estimatedTime = degreeToTurn/30.0 + 1 # Upon testing, the turning speed is 30 degree/second. Add one more second.
        print('{} - Absolute degree to turn is {} degree. Estimated time is {} seconds.'.format(time.ctime(), degreeToTurn, estimatedTime))
    
    # create the CONDITION_YAW command using command_long_encode()
    msg = drone.vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        yaw_inDegree,  # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, if set to 0, yaw is an absolute direction[0-360](0=north, 90=east); if set to 1, yaw is a relative degree to the current yaw direction.
        0, 0, 0)    # param 5 ~ 7 not used
    
    # Send MAVLink message.
    drone.vehicle.send_mavlink(msg)

def cu_lo(drone):
    point = drone.vehicle.location.global_relative_frame
    return point

def line(dis = 4, alt = 2):
    pointA = cu_lo(MCU)
    cdis = 0
    A = (pointA.lat, pointA.lon)
    cdis = cdis + dis
    B = new_coords(A,cdis,0)
    cdis = cdis + dis
    C = new_coords(A,cdis,0)
    cdis = cdis + dis
    D = new_coords(A,cdis,0)
    set_yaw(MCU,0)
    goto(CD1,B[0],B[1],alt)
    set_yaw(CD1,0)
    goto(CD2,C[0],C[1],alt)
    set_yaw(CD2,0)
    goto(CD3,D[0],D[1],alt)
    set_yaw(CD3,0)
    time.sleep(1)
    
    

def square(side = 8, alt = 3):
    pointA = cu_lo(MCU)
    mcuculo = (pointA.lat,pointA.lon)
    pointB = cu_lo(CD1)
    cd1culo = (pointB.lat,pointB.lon)
    pointC = cu_lo(CD2)
    cd2culo = (pointC.lat,pointC.lon)
    pointD = cu_lo(CD3)
    cd3culo = (pointD.lat,pointD.lon)

    cd1golo = new_coords(cd1culo, side, 90)
    goto(CD1,cd1golo[0],cd1golo[1],alt)

    cd3golo = new_coords(cd3culo, side, 90)
    goto(CD3,cd3golo[0],cd3golo[1],alt)

    cd1golo = new_coords(mcuculo, side, 90)
    goto(CD1,cd1golo[0],cd1golo[1],alt)

    cd3golo = new_coords(cd2culo, side, 90)
    goto(CD3,cd3golo[0],cd3golo[1],alt)

    set_yaw(MCU,0)
    set_yaw(CD1,0)
    set_yaw(CD2,0)
    set_yaw(CD3,0)

    time.sleep(2)

def tri(side=6,alt=3):
    pointA = cu_lo(CD1)
    A = (pointA.lat, pointA.lon)
    B = new_coords(A, side, 90)
    goto(CD3, B[0], B[1], alt)
    set_yaw(MCU,0)
    set_yaw(CD1,0)
    set_yaw(CD2,0)
    set_yaw(CD3,0)

def main():

    start_server_service(local_host)
    threading.Thread(target=ServerSendStatus, args=(MCU, local_host, status_port[0],)).start()
    print("SendStatus Active")
    threading.Thread(target=ServerSendStatus, args=(CD1, local_host, status_port[1],)).start()
    print("SendStatus Active")
    threading.Thread(target=ServerSendStatus, args=(CD2, local_host, status_port[2],)).start()
    print("SendStatus Active")
    threading.Thread(target=ServerSendStatus, args=(CD3, local_host, status_port[3],)).start()
    print("SendStatus Active")

if __name__ == "__main__":
    main()
