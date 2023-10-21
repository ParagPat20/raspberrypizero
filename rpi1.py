##### RPI1 MCU, CD1 #####
import RPI_Functions
from RPI_Functions import Drone
from dronekit import VehicleMode
import time

local_host = '0.0.0.0'
cmd_port = 12345
ctrl_port = 54321
st_port = 60001
status_waitForCommand = True

MCU = Drone('tcp:127.0.0.1:5762')
print("MCU connected")
CD1 = Drone('tcp:127.0.0.1:5772')
print("CD1 Connected")

drone1 = MCU
drone2 = CD1
Drone_ID = MCU

def cu_lo(drone):
    point = drone.vehicle.location.global_relative_frame
    return point

def LINE(dis = 2, alt = 1):
    pointA = cu_lo(MCU)
    cdis = 0
    A = (pointA.lat, pointA.lon)
    cdis = cdis + dis
    goto(MCU,A[0],A[1],alt,0.7)
    MCU.vehicle.mode = VehicleMode('POSHOLD')
    print("MCU Reached and Fixed on its Position")
    B = new_coords(A,cdis,0)
    cdis = cdis + dis
    goto(CD1,B[0],B[1],alt,0.7)
    CD1.vehicle.mode = VehicleMode('POSHOLD')
    print("CD Reached and Fixed on its Position")
    time.sleep(1)
    print("Line Completed")

start_server(local_host)
start_drone_server_services(MCU, local_host,60002)
start_drone_server_services(CD1, local_host,60003)
