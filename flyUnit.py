#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import math
from dronekit import connect, LocationGlobalRelative

class FlyUnit:
    def __init__(self, connect_string="/dev/ttyACM0"):
        try:
            self.vehicle = connect(connect_string, wait_ready=True, baud=115200)
            self.vehicle.wait_ready('autopilot_version')
            self.isConnect = True
            print("Connected")
        except Exception as e:
            self.isConnect = False
            print("Failed to connect:", str(e))
    
    def read_info_drone(self):
        return {
            "firmware": str(self.vehicle.version),
            "type": str(self.vehicle.version.release_type()),
            "mode": str(self.vehicle.mode.name),
            "battery": {
                "voltage": str(self.vehicle.battery.voltage),
                "current": str(self.vehicle.battery.current),
                "level": str(self.vehicle.battery.level)
            },
            "heartbeat": str(self.vehicle.last_heartbeat),
            "armed": self.vehicle.armed,
            "flagDisiableArmed": str(self.vehicle.is_armable),
            "status": str(self.vehicle.system_status.state)
        }

    def read_info_attitude(self):  # Fixed typo: 'read_info_atitude' to 'read_info_attitude'
        return {
            "attitude": {  # Fixed typo: 'atitude' to 'attitude'
                "pitch": float(format(self.vehicle.attitude.pitch, ".2f")),
                "roll": float(format(self.vehicle.attitude.roll, ".2f")),
                "yaw": float(format(self.vehicle.attitude.yaw, ".2f")),
            },
            "velocity": {
                "vx": self.vehicle.velocity[0],
                "vy": self.vehicle.velocity[1],
                "vz": self.vehicle.velocity[2],
            },
            "heading": self.vehicle.heading,
        }

    def read_info_gps(self):
        return {
            "sat": self.vehicle.gps_0.satellites_visible,
            "fix": self.vehicle.gps_0.fix_type,
            "Groundspeed": self.vehicle.groundspeed,
            "lat": self.vehicle.location.global_frame.lat,
            "lon": self.vehicle.location.global_frame.lon,
            "alt": self.vehicle.location.global_relative_frame.alt,
            "homeLocation": {
                "lat": self.vehicle.home_location.lat if self.vehicle.home_location is not None else 0,
                "lon": self.vehicle.home_location.lon if self.vehicle.home_location is not None else 0,
                "alt": self.vehicle.home_location.alt if self.vehicle.home_location is not None else 0,
            }
        }

    def distance_to_current_waypoint(self, waypoint):
        """
        Returns the ground distance in meters to the waypoint.
        """
        distance = self.get_distance_metres(
            [self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon], 
            [waypoint["lat"], waypoint["lng"]]
        )
        return distance
    
    @staticmethod
    def get_distance_metres(aLocation1, aLocation2):
        """
        Returns the ground distance in meters between two locations.
        """

        R = 6378.0  # Radius of Earth in kilometers

        lat1 = aLocation1[0]
        lat2 = aLocation2[0]

        lon1 = aLocation1[1]
        lon2 = aLocation2[1]

        # Convert latitude and longitude from degrees to radians
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        # Haversine formula
        a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        # Distance in kilometers
        distance = R * c

        distance_m = distance * 1000  # Convert to meters

        return distance_m
    
    def goto_waypoint(self, waypoint):
        """
        Commands the vehicle to go to a specified waypoint.
        """
        point = LocationGlobalRelative(
            waypoint["lat"], waypoint["lng"], waypoint["alt"]
        )
        self.vehicle.simple_goto(point, groundspeed=waypoint["speed"])

if __name__ == "__main__":
    # Create an instance of FlyUnit
    fly_unit = FlyUnit(connect_string="/dev/ttyACM0")

    if fly_unit.isConnect:
        try:
            while True:
                drone_info = fly_unit.read_info_drone()
                # print("Drone Information:", drone_info)
                
                attitude_info = fly_unit.read_info_attitude()
                # print("Attitude Information:", attitude_info)
                
                gps_info = fly_unit.read_info_gps()
                print("GPS Information:", gps_info)
        except:
            pass
    #     # Read and print drone information
    #     drone_info = fly_unit.read_info_drone()
    #     print("Drone Information:", drone_info)

    #     # Read and print attitude information
    #     attitude_info = fly_unit.read_info_attitude()
    #     print("Attitude Information:", attitude_info)

    #     # Read and print GPS information
    #     gps_info = fly_unit.read_info_gps()
    #     print("GPS Information:", gps_info)

    #     # Define a waypoint and calculate distance to it
    #     waypoint = {"lat": 37.7749, "lng": -122.4194, "alt": 100, "speed": 5}
    #     distance = fly_unit.distance_to_current_waypoint(waypoint)
    #     print("Distance to waypoint:", distance, "meters")

    #     # Command the vehicle to go to the waypoint
    #     fly_unit.goto_waypoint(waypoint)
    #     print("Sent command to go to waypoint:", waypoint)
    else:
        print("Vehicle not connected. Unable to perform operations.")
