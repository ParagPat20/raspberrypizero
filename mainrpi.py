from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import socket
import threading

local_host = '192.168.22.122'
remote_host = '192.168.22.122'
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
            0, 0, 0, # x, y, z velocity in m/s  (not used)
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

    def arm(self, mode):
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


# MCU = Drone('/dev/serial0',baudrate=115200)
# print("MCU connected")
# CD1 = Drone('0.0.0.0:14550')
# print("CD1 Connected")
# CD2 = Drone('0.0.0.0:14552')
# print("CD2 Connected")
# CD3 = Drone('0.0.0.0:14553')
# print("CD3 Connected")
MCU = Drone('tcp:127.0.0.1:5762')
print("MCU connected")
CD1 = Drone('tcp:127.0.0.1:5772')
print("CD1 Connected")
CD2 = Drone('tcp:127.0.0.1:5782')
print("CD2 Connected")
CD3 = Drone('tcp:127.0.0.1:5792')
print("CD3 Connected")
Drone_ID = MCU

def ServerSendStatus(drone, local_host, status_port):

    status_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    status_socket.bind((local_host, status_port))
    status_socket.listen(1)

    print('{} - ServerSendStatus is started!'.format(time.ctime()))

    while True:
        client_connection, client_address = status_socket.accept()

        status_data = status(drone)  # Call your status function here
        client_connection.send(status_data.encode())  # Send the string

        client_connection.close()
        time.sleep(1)


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

    status_str = ','.join([b, gs, md, vx,vy,vz, gps, lat, lon, alt])

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
        if immediate_command_str == 'takeoff_all':
            drone_takeoff(MCU)
            drone_takeoff(CD1)
            drone_takeoff(CD2)
            drone_takeoff(CD3)
        if immediate_command_str == 'square':
            drone_ctrl(MCU,1,0,0)
            drone_ctrl(CD2,1,0,0)
            print("Square1")
            time.sleep(8)
            drone_ctrl(MCU,0,0,0)
            drone_ctrl(CD2,0,0,0)
            print("Square2")
            time.sleep(8)
            drone_ctrl(MCU,-1,0,0)
            drone_ctrl(CD2,-1,0,0)
            print("Square3")
            time.sleep(8)
            drone_ctrl(MCU,0,0,0)
            drone_ctrl(CD2,0,0,0)
            print("Square4")
            time.sleep(8)
        if immediate_command_str == 'POSHOLD':
            drone_mode(MCU,'POSHOLD')
            drone_mode(CD1,'POSHOLD')
            drone_mode(CD2,'POSHOLD')
            drone_mode(CD3,'POSHOLD')
        if immediate_command_str == 'squarevel':
            drone_vel_ctrl(MCU,-0.8,0,0,5)
            drone_vel_ctrl(CD2,-0.8,0,0,5)
            print("Square1")
            time.sleep(8)
            drone_vel_ctrl(MCU,0.8,0,0,5)
            drone_vel_ctrl(CD2,0.8,0,0,5)
            print("Square2")
            time.sleep(8)

            


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
            drone_vel_ctrl(MCU,x,y,z,1)
            drone_vel_ctrl(CD1,x,y,z,1)
            drone_vel_ctrl(CD2,x,y,z,1)
            drone_vel_ctrl(CD3,x,y,z,1)
        except ValueError:
            print("Invalid control command format. Expected 'x,y,z'")

        client_connection.close()

def drone_ctrl(drone,x,y,z):
    threading.Thread(target=drone.position_target_local_ned, args=(x, y, z,)).start()

def drone_vel_ctrl(drone,x,y,z,t):
    threading.Thread(target=drone.send_ned_velocity, args=(x, y, z, t)).start()

def drone_takeoff(drone):
    threading.Thread(target=drone.takeoff).start()

def drone_arm(drone):
    threading.Thread(target=drone.arm, args=('STABILIZE')).start()

def drone_land(drone):
    threading.Thread(target=drone.land).start()

def set_mode(drone,mode):
    drone.vehicle.mode = VehicleMode(mode)

def drone_mode(drone,mode):
    threading.Thread(target=set_mode, args= (drone,mode)).start

def start_server_service(local_host):
    threading.Thread(target=ServerRecvCmd, args=(local_host,)).start()
    threading.Thread(target=ServerRecvControl, args=(local_host,)).start()
    print("Thread serverRecvMode and ServerRecvControl are started!")


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
