from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import socket
import threading
import os

local_host = '0.0.0.0'
remote_host = '192.168.14.101'
mode_port = 60001
ctrl_port = 60003
status_port = [60002,60004,60006,60008]
stop = 0

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

    def stop(self):
        global stop
        stop = 1
        time.sleep(3)
        stop = 0

MCU = Drone('tcp:127.0.0.1:5762')
CD1 = Drone('tcp:127.0.0.1:5772')
CD2 = Drone('tcp:127.0.0.1:5782')
CD3 = Drone('tcp:127.0.0.1:5792')
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
            Drone_ID.arm('GUIDED')
        if immediate_command_str == 'LAND':
            Drone_ID.land()
        if immediate_command_str == 'TakeOff':
            Drone_ID.takeoff()

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
            Drone_ID.send_ned_velocity(x, y, z)  # Send NED velocity commands to the drone
        except ValueError:
            print("Invalid control command format. Expected 'x,y,z'")

        client_connection.close()

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
