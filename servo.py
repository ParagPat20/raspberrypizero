from dronekit import connect
from pymavlink import mavutil
import time

d1 = connect("COM14")
print("Set Ch2 override to 200 (indexing syntax)")
i = 1000
while True:
    d1.channels.overrides['5'] = 1000
    print(" Ch2 override: %s" % d1.channels.overrides['5'])
    time.sleep(1)
    d1.channels.overrides['5'] = 1600
    print(" Ch2 override: %s" % d1.channels.overrides['5'])
    time.sleep(1)