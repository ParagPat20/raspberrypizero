#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.

channel_overrides.py: 

Demonstrates how set and clear channel-override information.

# NOTE: 
Channel overrides (a.k.a "RC overrides") are highly discommended (they are primarily implemented 
for simulating user input and when implementing certain types of joystick control).

They are provided for development purposes. Please raise an issue explaining why you need them
and we will try to find a better alternative: https://github.com/dronekit/dronekit-python/issues

Full documentation is provided at http://python.dronekit.io/examples/channel_overrides.html
"""
from __future__ import print_function
from dronekit import connect


#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Example showing how to set and clear vehicle channel-override information.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
while True:
    pass