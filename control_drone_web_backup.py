#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Import library for project
"""


from __future__ import print_function
from dronekit import VehicleMode, LocationGlobalRelative

import subprocess
import queue

import time

from flyUnit import FlyUnit
from camera import Camera
from camera_usb import CameraUsb
from control import *
from socket_io import *
from stream import RTSPStreamer
from sensor import *
from qr import *
import threading

"""
Defined variable for project
"""
DELAY_SEND_INFO = .6
DELAY_ACTION = .8

time_prev_send_info = 0
time_prev_action = 0

time_curr = 0

"""
Class Drone to handle event of drone to project
"""

class Drone:
    #1270, 720 - 1538, 864
    def __init__(self, video_filename="output_15_8_2024_2.mp4", server_url = 'http://103.167.198.50:5000', key_stream="albert3", size_frame=(1270, 720), codec='libx264', bitrate='1M', preset='ultrafast'):
        
        self.copter = FlyUnit()

        self.servo = Servo()
        
        self.buttonCoffee = ButtonReader(pin=27)
        
        self.sensorPerson1 = LD2410B(pin=5)
        self.sensorPerson2 = LD2410B(pin=6)
        
        self.ob_qr = ObjectAndQrDetector()
        self.data_qr = None
        self.data_objectPerson = 0
        
        
        self.running = True
        self.picam = CameraUsb(resolution=size_frame,  video_filename=video_filename)
        # self.picam.set_continuous_autofocus()
        self.frame = None

        self.frame_queue = queue.Queue(maxsize=1)  # A queue to hold one frame at a time
        
        self.streamer = RTSPStreamer(
            rtsp_url='rtmp://103.167.198.50/live/' + key_stream,
            width=size_frame[0],
            height=size_frame[1],
            frame_rate=10,
            codec=codec,
            bitrate=bitrate,
            preset=preset
        )
        
        self.socket_client = Socket(server_url)
        self.socket_client.register_events()
        self.socket_client.sio.on('drone', self.handle_drone_event)

        self.set_one_time = False

        self.waypoints = None
        self.waypoint_index = 0
        self.waypoint_index_recv = None
        self.flag_change_dis = False
        self.current_waypoint = None
        self.isFinshedWp = False
        self.isFinshedCoffee = False
        self.positionCoffee = False
        self.isRunCoffee = False
        self.isRunWp = None
        self.isPerson = True
        
        self.time_send_socket_prev = 0
        self.DELAY_SEND_SOCKET = 1

        try:
            self.socket_client.connect()

            if self.copter.isConnect:
                self.droneVehicle = self.copter.vehicle

        except Exception as e:
            print("error in duration connect to server")

    def handle_drone_event(self, payload):
        print(f"Drone event received with header: {payload['header']} and data: {payload['data']}")
        mode = str(self.droneVehicle.mode.name)
        if self.socket_client.isConnect:
            if mode == "GUIDED":
                if payload['header'] == "arm":
                    if payload["data"] is True:
                        self.droneVehicle.armed = True
                        while not self.droneVehicle.armed:
                            if str(self.droneVehicle.mode.name) != "GUIDED":
                                break
                            time.sleep(.1)
                            self.socket_client.send_message("controlMsg", "web", "droneStatus", "drone is armming...")
                        self.socket_client.send_message("controlMsg", "web", "droneStatus", "drone is armmed")
                    elif payload["data"] is False:
                        self.droneVehicle.armed = False
                        self.socket_client.send_message("controlMsg", "web", "droneStatus","drone is disarmming...")

                if payload["header"] == "takeoff":
                    if payload["data"]["ctrl"] is True:
                        self.droneVehicle.simple_takeoff(int(payload['data']['alt']))
                        self.socket_client.send_message("controlMsg", "web", "droneStatus",f"drone is takeoff with distance is: {payload['data']['alt']}")

                if payload["header"] == "land":
                    if payload["data"] == True:
                        print("Landing...")
                        self.droneVehicle.mode = VehicleMode("LAND")
                        while self.droneVehicle.armed:
                            print("Waiting for disarming...")
                            self.socket_client.send_message("controlMsg", "web", "droneStatus","Waiting for disarming...")
                            time.sleep(1)
                        self.droneVehicle.mode = VehicleMode("GUIDED")
                        print("Landed and disarmed")
                        self.socket_client.send_message("controlMsg", "web", "droneStatus","Landed and disarmed")

                if payload["header"] == "set_home_gps":
                    print(f"set home gps: {payload['data']}")
                    my_location_alt = self.droneVehicle.location.global_frame
                    my_location_alt.lat = payload['data']['lat']
                    my_location_alt.lon = payload['data']['lon']
                    my_location_alt.alt = payload['data']['alt']
                    self.droneVehicle.home_location = my_location_alt
                    self.socket_client.send_message("controlMsg", "web", "droneStatus", f"set new home: {payload['data']['lat']} - {payload['data']['lon']} - {payload['data']['alt']}")

                if payload["header"] == "run_wp":

                    self.waypoint_index = 0
                    self.waypoint_index_recv = None
                    self.flag_change_dis = False
                    self.current_waypoint = None
                    self.isFinshedWp = False
                    self.isFinshedCoffee = False
                    self.positionCoffee = False

                    point = LocationGlobalRelative(payload['data']['lat'], payload['data']['lng'], int(payload['data']['alt']))
                    self.droneVehicle.simple_goto(point, groundspeed=int(payload['data']['speed']))
                    self.socket_client.send_message("controlMsg", "web", "droneStatus", f"drone wp to {payload['data']['lat']} - {payload['data']['lng']} - {payload['data']['speed']}(m/s)")

                if payload["header"] == "run_all_wp":
                    self.waypoints = payload["data"]
                    self.socket_client.send_message("controlMsg", "web", "droneStatus", f"load run all wp with {len(self.waypoints)} wps")

                if payload["header"] == "run_start_wp":
                    print(f"start all wp: {payload['data']}")
                    self.isRunWp = payload['data']

                    self.waypoint_index = 0
                    self.waypoint_index_recv = None
                    self.flag_change_dis = False
                    self.current_waypoint = None
                    self.isFinshedWp = False
                    self.isFinshedCoffee = False
                    self.positionCoffee = False
                    
                    self.socket_client.send_message("controlMsg", "web", "droneStatus", f"{self.isRunWp} all wp")

                if payload["header"] == "command":
                    print(f"command is: {payload['data']}")

                if payload["header"] == "returnToHome":
                    print(f"returnToHome is: {payload['data']}")
                    if payload['data'] == True:
                        self.droneVehicle.mode = VehicleMode("RTL")
                    else:
                        self.droneVehicle.mode = VehicleMode("GUIDED")

                if payload["header"] == "runCoffee":
                    self.isRunCoffee = payload['data']
                    print(f"runCoffee is: {payload['data']}")


                if payload["header"] == "camera":
                    if payload["data"] == 'on':
                        print(f"camera is on")
                        self.picam.isRunCamera = True
                    elif payload["data"] == 'off':
                        print(f"camera is off")
                        self.picam.isRunCamera = False

                if payload["header"] == "focusMode":
                    if payload['data'] == 'manual':
                        print(f"focusMode is manual")
                        self.picam.set_manual_focus(1.0)
                    elif payload['data'] == 'auto':
                        print(f"focusMode is auto")
                        self.picam.set_continuous_autofocus()

                if payload["header"] == "lensFocus":
                    self.picam.set_manual_focus(float(payload['data']))
                    print(f"focusMode is manual with lenFocus: {payload['data']}")

                if payload["header"] == "zoom":
                    self.picam.set_zoom(float(payload['data']))
                    print(f"zoom is : {payload['data']}")

                if payload["header"] == "servo":
                    if payload['data'] == 'open':
                        self.servo.open_handle()
                    elif payload['data'] == 'close':
                        self.servo.close_handle()

            else:
                time.sleep(2)
                self.socket_client.send_message("controlMsg", "web", "droneStatus", "change mode GUIDED in tx, please!!!")

    def processing_1(self):
        """
            Processing_1 to read camera and stream camera to server
        """
        self.streamer.start_stream()
        while self.running:
            if self.picam.isRunCamera or True:
                self.frame = self.picam.read_camera()
                if self.frame is not None:
                    self.streamer.process.stdin.write(self.frame.tobytes())
                    self.picam.video_writer.write(self.frame)

    def processing_2(self):
        while self.running:
            if self.frame is not None:
                # Process frame if necessary
                self.frame, data_qr = self.ob_qr.qr_detect(self.frame)
                self.frame, self.data_person = self.ob_qr.object_detect(self.frame)
                
                print(f"  data_qr: {data_qr}    -    data_person: {self.data_person}")
                if time.time() - self.time_send_socket_prev > self.DELAY_SEND_SOCKET:
                    self.time_send_socket_prev = time.time()
                    if len(data_qr) != 0:
                        self.data_qr = data_qr[0]
                        self.socket_client.send_message("controlMsg", "web", "droneFlyStatus", f"QR Detected: {data_qr[0]}")
                    else:
                        self.data_qr = None
                        self.socket_client.send_message("controlMsg", "web", "droneFlyStatus", f"QR none Detected")

    def start(self):
        self.thread_1 = threading.Thread(target=self.processing_1, daemon=True)
        self.thread_2 = threading.Thread(target=self.processing_2, daemon=True)
        self.thread_1.start()
        self.thread_2.start()

    def stop(self):
        self.running = False
        self.thread_1.join()
        self.thread_2.join()
        self.streamer.stop_stream()
        self.socket_client.disconnect()
        self.picam.release()
        self.servo.servo_clean()
        self.buttonCoffee.cleanup()
        self.sensorPerson1.cleanup()
        self.sensorPerson2.cleanup()

def main():

    global time_curr, time_prev_send_info, time_prev_action, DELAY_ACTION, DELAY_SEND_INFO

    drone_instance = Drone()
    drone_instance.start()

    try:
        while drone_instance.copter.isConnect:
            # Main thread can handle other tasks here
            time_curr = time.time()
            
            # print(f"{time_curr - time_prev_send_info} - {time_curr - time_prev_action}")
            
            if time_curr - time_prev_send_info > DELAY_SEND_INFO:
                time_prev_send_info = time_curr

                payload_info_drone = drone_instance.copter.read_info_drone()
                drone_instance.socket_client.send_message("controlMsg", "web","droneStatusInfor", payload_info_drone)

                payload_info_atitude = drone_instance.copter.read_info_attitude()
                drone_instance.socket_client.send_message("controlMsg", "web","droneStatusAtitude", payload_info_atitude)

                payload_info_gps = drone_instance.copter.read_info_gps()
                drone_instance.socket_client.send_message("controlMsg", "web","droneStatusGps", payload_info_gps)

            if time_curr - time_prev_action > DELAY_ACTION:
                time_prev_action = time_curr

                drone_instance.isPerson = drone_instance.sensorPerson1.read_LD2410B() or drone_instance.sensorPerson2.read_LD2410B()

                if drone_instance.isRunWp is not None and drone_instance.waypoints is not None:
                    if str(drone_instance.droneVehicle.mode.name) == "GUIDED" and drone_instance.isRunWp == True:
                        if drone_instance.waypoint_index < len(drone_instance.waypoints):
                            drone_instance.current_waypoint = drone_instance.waypoints[drone_instance.waypoint_index]
                            if drone_instance.waypoint_index_recv != drone_instance.current_waypoint:
                                drone_instance.flag_change_dis = False
                                drone_instance.waypoint_index_recv = drone_instance.current_waypoint
                                drone_instance.copter.goto_waypoint(drone_instance.current_waypoint)
                            else:
                                distance = drone_instance.copter.distance_to_current_waypoint(drone_instance.current_waypoint)
                                print(f"Distance to waypoint: {distance:.2f} meters, waypoint: {drone_instance.waypoint_index}")
                                if distance < 1:
                                    if drone_instance.flag_change_dis is False:
                                        drone_instance.flag_change_dis = True
                                        print("Waypoint reached!")
                                        drone_instance.waypoint_index = drone_instance.waypoint_index + 1
                    print(f"  cc:   {drone_instance.waypoint_index} - {len(drone_instance.waypoints)}")
                    if drone_instance.waypoint_index == len(drone_instance.waypoints):
                        drone_instance.isFinshedWp = True
                        

                if drone_instance.isFinshedWp:
                    if drone_instance.isFinshedCoffee is False:
                        """
                            read qr code to handle catch coffee cup
                        """
                        if drone_instance.data_qr is not None and drone_instance.data_qr == "2" :
                            drone_instance.positionCoffee = True
                            
                        if drone_instance.positionCoffee or 1:
                            """
                                QR detected -> open servo -> landing -> disarm
                            """
                            drone_instance.servo.open_handle()


                            drone_instance.droneVehicle.mode = VehicleMode("LAND")
                            
                            while drone_instance.droneVehicle.armed:
                                 # print("Waiting for disarming...")
                                 # drone_instance.socket_client.send_message("controlMsg", "web", "droneStatus","Waiting for disarming...")
                                 time.sleep(1)
                            
                            if not drone_instance.droneVehicle.armed:
                                drone_instance.droneVehicle.mode = VehicleMode("GUIDED")

                            print("Landed and disarmed")
                            drone_instance.socket_client.send_message("controlMsg", "web", "droneStatus","Landed and disarmed")
                            
                            """
                                waiting to person give coffee cup
                            """
                            if drone_instance.buttonCoffee.read_button() == True:
                                """
                                    when have person to give coffee for drone -> close servo
                                """
                                
                                drone_instance.servo.close_handle()
                                drone_instance.isFinshedCoffee = True
                    else:
                        if not drone_instance.isPerson:
                            """
                                when no person -> arm -> takeoff -> return to home
                            """
                            drone_instance.droneVehicle.armed = True
                            
                            while not drone_instance.droneVehicle.armed:
                                time.sleep(.1)
                            
                            drone_instance.droneVehicle.simple_takeoff(30)
                            

                           # drone_instance.droneVehicle.mode = VehicleMode("RTL")

                        
#                print(f"         {drone_instance.sensorPerson1.read_LD2410B()} - {drone_instance.sensorPerson2.read_LD2410B()} - {drone_instance.sensorPerson3.read_LD2410B()}")
                # print(f"                                    data QR: {drone_instance.data_qr} - btn: {drone_instance.buttonCoffee.read_button()} - person: {drone_instance.isPerson}")

    except KeyboardInterrupt:
        drone_instance.stop()

if __name__ == "__main__":
    main()
