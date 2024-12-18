from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import geopy
import geopy.distance
from geopy.distance import great_circle

class Drone:
    def __init__(self, connection_string):
        self.vehicle = connect(connection_string)
        # Example: drone = Drone(connection_string="udp:127.0.0.1:14551")

    def disconnect(self):
        self.vehicle.close()
        self.vehicle = None
        time.sleep(2)
        print("Disconnected Successfully")
        # Example: drone.disconnect()

    def arm(self, mode='GUIDED'):
        """Arms the vehicle in specified mode"""
        print("Arming motors")
        self.vehicle.mode = VehicleMode(mode)
        self.vehicle.armed = True
        
        while not self.vehicle.armed:
            print("Waiting for arming...")
            self.vehicle.armed = True
            time.sleep(1)
        print("Vehicle Armed")
        # Example: drone.arm(mode='GUIDED')

    def takeoff(self, alt=1):
        """Takes off to specified altitude"""
        self.arm()
        print("Taking off!")
        self.vehicle.simple_takeoff(alt)
        start_time = time.time()
        TIMEOUT_SECONDS = 15
        
        while True:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            print(f" Altitude: {current_altitude}")
            if current_altitude >= alt * 0.95:
                print("Reached target altitude")
                break
            else:
                print("Waiting for altitude information...")
            if time.time() - start_time > TIMEOUT_SECONDS:
                break
            time.sleep(1)
        # Example: drone.takeoff(alt=10)  # Take off to 10 meters

    def land(self):
        """Switches to LAND mode"""
        self.vehicle.mode = VehicleMode("LAND")
        print("Landing")
        # Example: drone.land()

    def rtl(self):
        """Return to launch"""
        self.vehicle.mode = VehicleMode("RTL")
        print("Returning to Launch")
        # Example: drone.rtl()  # Return to launch

    def poshold(self):
        """Switches to POSHOLD mode"""
        self.vehicle.mode = VehicleMode("POSHOLD")
        print("Position Hold mode enabled")
        # Example: drone.poshold()  # Enable position hold mode

    def disarm(self):
        try:
            print("Disarming motors")
            self.vehicle.armed = False

            while self.vehicle.armed:
                print("Waiting for disarming...")
                self.vehicle.armed = False
                time.sleep(1)

            print("Vehicle Disarmed")
        except Exception as e:
            print(f"Error during disarming: {e}")
        # Example: drone.disarm()

    def get_location(self):
        """Returns current location and heading"""
        lat = self.vehicle.location.global_relative_frame.lat
        lon = self.vehicle.location.global_relative_frame.lon
        heading = self.vehicle.heading
        return (lat, lon), heading
        # Example: location, heading = drone.get_location()

    def goto(self, location, alt, groundspeed=0.7):
        """Goes to specified GPS location and altitude"""
        destination = LocationGlobalRelative(location[0], location[1], alt)
        
        current_lat = self.vehicle.location.global_relative_frame.lat
        current_lon = self.vehicle.location.global_relative_frame.lon
        current_alt = self.vehicle.location.global_relative_frame.alt
        
        while ((self.distance_between_points((current_lat, current_lon), location) > 0.5) or 
                (abs(current_alt - alt) > 0.3)):
            self.vehicle.simple_goto(destination, groundspeed=groundspeed)
            time.sleep(0.5)
            current_lat = self.vehicle.location.global_relative_frame.lat
            current_lon = self.vehicle.location.global_relative_frame.lon
            current_alt = self.vehicle.location.global_relative_frame.alt
        # Example: drone.goto(location=(37.7749, -122.4194), alt=10)  # Go to specified GPS location

    def send_ned_velocity_drone(self, velocity_x, velocity_y, velocity_z):
        try:
            velocity_x = float(velocity_x)
            velocity_y = float(velocity_y)
            velocity_z = float(velocity_z)
            

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
            print("Drone Velocity : {}, {}, {}".format(velocity_x,velocity_y,velocity_z))

        except Exception as e:
            print(f"Error sending velocity commands: {e}")
        # Example: drone.send_ned_velocity_drone(1, 0, 0)  # Move forward at 1 m/s

    def send_ned_velocity(self, x, y, z, duration = None):
        self.no_vel_cmds = False
        if duration:
            for i in range(0,duration):
                self.send_ned_velocity_drone(x,y,z)
                print(i)
                time.sleep(1)

            self.send_ned_velocity_drone(0,0,0)
            time.sleep(1)
            self.no_vel_cmds = True
            
        else:
            self.send_ned_velocity_drone(x,y,z)
        # Example: drone.send_ned_velocity(1, 0, 0, duration=5)  # Move forward for 5 seconds

    def yaw(self, heading, relative=False):
        """
        Rotate vehicle to specified heading
        heading: degrees (0 is North)
        relative: if True, heading is relative to current heading
        """
        if relative:
            is_relative = 1
        else:
            is_relative = 0
            
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
            0,       # confirmation
            heading, # param 1, yaw in degrees
            0,       # param 2, yaw speed deg/s
            1,       # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0) # param 5 ~ 7 not used
        
        self.vehicle.send_mavlink(msg)
        # Example: drone.yaw(heading=90, relative=True)  # Rotate 90 degrees relative to current heading

    def get_vehicle_state(self):
        """Returns comprehensive vehicle state information"""
        return {
            'location': {
                'global': (self.vehicle.location.global_frame.lat,
                            self.vehicle.location.global_frame.lon,
                            self.vehicle.location.global_frame.alt),
                'relative': (self.vehicle.location.global_relative_frame.lat,
                            self.vehicle.location.global_relative_frame.lon,
                            self.vehicle.location.global_relative_frame.alt),
                'local': (self.vehicle.location.local_frame.north,
                            self.vehicle.location.local_frame.east,
                            self.vehicle.location.local_frame.down)
            },
            'velocity': self.vehicle.velocity,
            'gps': {
                'fix_type': self.vehicle.gps_0.fix_type,
                'satellites_visible': self.vehicle.gps_0.satellites_visible
            },
            'battery': {
                'voltage': self.vehicle.battery.voltage,
                'current': self.vehicle.battery.current,
                'level': self.vehicle.battery.level
            },
            'heading': self.vehicle.heading,
            'groundspeed': self.vehicle.groundspeed,
            'airspeed': self.vehicle.airspeed,
            'mode': self.vehicle.mode.name,
        }
        # Example: state = drone.get_vehicle_state()

    def set_mode(self, mode_name):
        """Changes vehicle mode"""
        self.vehicle.mode = VehicleMode(mode_name)
        print(f"Mode changed to {mode_name}")
        # Example: drone.set_mode('LOITER')  # Change mode to LOITER


    def close_vehicle(self):
        """Closes vehicle connection"""
        self.vehicle.close()
        # Example: drone.close_vehicle()


    def move_to_location(self, distance, altitude, direction_degree):
        """
        Move the drone to a new location based on distance, altitude, and direction.
        
        :param distance: Distance to move in meters.
        :param altitude: Altitude to maintain in meters.
        :param direction_degree: Direction in degrees (0 is North, 90 is east).
        """
        try:
            # Get the current GPS location of the drone
            current_location = (self.vehicle.location.global_relative_frame.lat, 
                                self.vehicle.location.global_relative_frame.lon)

            # Calculate the new coordinates based on distance and direction
            new_location = new_coords(current_location, distance, direction_degree)

            # Calculate the distance to the new location
            distance_to_new_location = calculate_distance(current_location, new_location)
            distance_to_new_location1 = distance_between_two_gps_coord(current_location, new_location)

            # print the intended movement
            print("{} moving to new location {} at altitude {}m with direction {} degrees".format(self.name, new_location, altitude, direction_degree))

            # Check if the new location is within 10 meters
            if distance_to_new_location <= 10 and distance_to_new_location1 <= 10:
                # Command the drone to go to the new location at the specified altitude
                self.goto(new_location, altitude)
            else:
                print("{} Move to Location Error: New location is too far ({} meters)".format(self.name, distance_to_new_location))

        except Exception as e:
            print("{} Move to Location Error: {}".format(self.name, e))
        # Example: drone.move_to_location(distance=100, altitude=10, direction_degree=90)

    def rc_ov(self, mode, ch1=0, ch2=0, ch3=0, ch4=0, ch5=0, ch6=0):
        """
        Override RC channels.
        
        :param mode: RC mode to override
        :param ch1: Value for RC channel 1
        :param ch2: Value for RC channel 2
        :param ch3: Value for RC channel 3
        :param ch4: Value for RC channel 4
        :param ch5: Value for RC channel 5
        :param ch6: Value for RC channel 6
        """
        try:
            if mode == 1:
                # Set the RC channel overrides
                self.vehicle.channels.overrides = {
                    '1': ch1,
                    '2': ch2,
                    '3': ch3,
                    '4': ch4,
                    '5': ch5,
                    '6': ch6
                }

                print(f"RC channels overridden: {ch1}, {ch2}, {ch3}, {ch4}, {ch5}, {ch6}")
            else:
                self.vehicle.channels.overrides = {
                    '1': None,
                    '2': None,
                    '3': None,
                    '4': None,
                    '5': None,
                    '6': None
                }
        except Exception as e:
            print(f"Error overriding RC channels: {e}")
        # Example: drone.rc_ov(mode=1, ch1=1500, ch2=1500, ch3=1500, ch4=1500)

#################################################################################################################

def calculate_distance(location1, location2):
    """
    Calculate the distance between two GPS coordinates.
    
    :param location1: Tuple of (latitude, longitude) for the first location.
    :param location2: Tuple of (latitude, longitude) for the second location.
    :return: Distance in meters.
    """
    from geopy.distance import geodesic
    return geodesic(location1, location2).meters

def distance_between_two_gps_coord(point1, point2):
    try:
        distance = great_circle(point1, point2).meters
        return distance
    except Exception as e:
        print(f"Error calculating distance between two GPS coordinates: {e}")

def new_coords(original_gps_coord, displacement, rotation_degree_relative):
    try:
        vincentyDistance = geopy.distance.distance(meters=displacement)
        original_point = geopy.Point(original_gps_coord[0], original_gps_coord[1])
        new_gps_coord = vincentyDistance.destination(point=original_point, bearing=rotation_degree_relative)
        new_gps_lat = new_gps_coord.latitude
        new_gps_lon = new_gps_coord.longitude

        return (round(new_gps_lat, 7), round(new_gps_lon, 7))
    except Exception as e:
        print(f"Error in calculating new coordinates: {e}")
