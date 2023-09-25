from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import socket

C = {'Drone':0, 'vx': 0, 'vy': 0, 'vz': 0, 'Arming': 0, 'Mode': 'GUIDED', 'Takeoff': 0, 'mstart': False}
P = {
    1:{'Batt': 0, 'Groundspeed': 0, 'ARM': 0, 'GPS': 0, 'Altitude': 0, 'MODE': None, 'VelocityX': 0, 'VelocityY': 0, 'VelocityZ': 0},
    2:{'Batt': 0, 'Groundspeed': 0, 'ARM': 0, 'GPS': 0, 'Altitude': 0, 'MODE': None, 'VelocityX': 0, 'VelocityY': 0, 'VelocityZ': 0}
}

if C['Drone'] == 1:
    print("D1 initializing")
    D1 = connect('tcp:127.0.0.1:5762')
    print("D1 Initialized")

if C['Drone'] == 2:
    print("D2 initializing")
    D2 = connect('tcp:127.0.0.1:5772')
    print("D2 Initialized")

def control(controling_drone):
    if C['Arming'] == 1:
        controling_drone.arm()

    if C['Takeoff'] == 1:
        controling_drone.arm()
        controling_drone.takeoff()
    
    if P[controling_drone]['MODE'] != 'VehicleMode:' + C['Mode']:
        controling_drone.vehicle.mode = VehicleMode(C['Mode'])

    controling_drone.send_ned_velocity(C['vx'], C['vy'], C['vz'], 1)

while True:
    if C['mstart'] == True:
        if C['Drone'] == 1:
            control(D1)
        if C['Drone'] == 2:
            control(D2)
        if C['Drone'] == 3:
            control(D1)
            control(D2)
        