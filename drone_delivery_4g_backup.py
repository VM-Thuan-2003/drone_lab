#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Import library for project
"""


from __future__ import print_function
from dronekit import VehicleMode, LocationGlobalRelative

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
        self.buzzer = Buzzer(buzzer_pin=21)
        self.buttonCoffee = ButtonReader(pin=27)
        
        self.sensorPerson1 = LD2410B(pin=5)
        self.sensorPerson2 = LD2410B(pin=6)
        
        self.ob_qr = ObjectAndQrDetector()
        self.data_qr = None
        self.data_objectPerson = 0
        
        self.running = True
        self.picam = CameraUsb(resolution=size_frame,  video_filename=video_filename)
        self.frame = None
        self.frame1 = None
        
        self.streamer = RTSPStreamer(
            rtsp_url='rtmp://103.167.198.50/live/' + key_stream,
            width=size_frame[0],
            height=size_frame[1],
            frame_rate=30,
            codec=codec,
            bitrate=bitrate,
            preset=preset
        )
        
        self.socket_client = Socket(server_url)
        self.socket_client.register_events()
        self.socket_client.sio.on('drone', self.handle_drone_event)

        self.set_one_time = False
        self.set_one_time_home = False

        self.waypoints = None
        self.waypoint_index = 0
        self.waypoint_index_home = 0
        self.waypoint_index_recv = None
        self.flag_change_dis = False
        self.current_waypoint = None
        self.isFinshedWp = False
        self.isFinshedCoffee = False
        self.positionCoffee = False
        self.isPerson = True
        
        self.isRunWp = False
        self.isRunCoffee = False
        self.isRunHome = False
        self.returnCoffee = False
        self.isreturnHome = False
        
        self.time_send_socket_prev = 0
        self.DELAY_SEND_SOCKET = 1

        try:
            self.socket_client.connect()

            if self.copter.isConnect:
                self.droneVehicle = self.copter.vehicle

        except Exception as e:
            print(f"error in duration connect to server: {e}")

    def handle_payload_guided(self, header, data):
        """
            _summary_

        Args:
            header (_type_): _description_
            data (_type_): _description_
        """
        
        if header == "arm":
            if data is True:
                self.droneVehicle.armed = True
                while not self.droneVehicle.armed:
                    if str(self.droneVehicle.mode.name) != "GUIDED":
                        break
                    time.sleep(.1)
                    self.socket_client.send_message("controlMsg", "web", "droneStatus", "drone is armming...")
                self.socket_client.send_message("controlMsg", "web", "droneStatus", "drone is armmed")
            elif data is False:
                self.droneVehicle.armed = False
                self.socket_client.send_message("controlMsg", "web", "droneStatus","drone is disarmming...")
        elif header == "takeoff":
            if data["ctrl"] is True:
                self.droneVehicle.simple_takeoff(int(data['alt']))
                self.socket_client.send_message("controlMsg", "web", "droneStatus",f"drone is takeoff with distance is: {data['alt']}")
        elif header == "land":
            if data == True:
                print("Landing...")
                self.droneVehicle.mode = VehicleMode("LAND")
                while self.droneVehicle.armed:
                    print("Waiting for disarming...")
                    self.socket_client.send_message("controlMsg", "web", "droneStatus","Waiting for disarming...")
                    time.sleep(1)
                self.droneVehicle.mode = VehicleMode("GUIDED")
                print("Landed and disarmed")
                self.socket_client.send_message("controlMsg", "web", "droneStatus","Landed and disarmed")
        elif header == "returnToHome":
            print(f"returnToHome is: {data}")
            if data == True:
                self.droneVehicle.mode = VehicleMode("RTL")
            else:
                self.droneVehicle.mode = VehicleMode("GUIDED")
        elif header == "run_wp":
            
            self.waypoint_index = 0
            self.waypoint_index_recv = None
            self.flag_change_dis = False
            self.current_waypoint = None
            self.isFinshedWp = False
            self.isFinshedCoffee = False
            self.positionCoffee = False
            self.isRunHome = False
            self.returnCoffee = False
            self.isreturnHome = False
            
            point = LocationGlobalRelative(data['lat'], data['lng'], int(data['alt']))
            self.droneVehicle.simple_goto(point, groundspeed=int(data['speed']))
            self.socket_client.send_message("controlMsg", "web", "droneStatus", f"drone wp to {data['lat']} - {data['lng']} - {data['speed']}(m/s)")
        elif header == "run_start_wp":
            print(f"start all wp: {data}")
            self.isRunWp = data
            
            self.waypoint_index = 0
            self.waypoint_index_recv = None
            self.flag_change_dis = False
            self.current_waypoint = None
            self.isFinshedWp = False
            self.isFinshedCoffee = False
            self.positionCoffee = False
            self.isRunHome = False
            self.returnCoffee = False
            self.isreturnHome = False
            
            self.socket_client.send_message("controlMsg", "web", "droneStatus", f"{self.isRunWp} all wp")
        elif header == "runCoffee":
            self.isRunCoffee = data
            print(f"runCoffee is: {data}")
        else:
            pass
    
    def handle_payload_otherMode(self, header, data):
        """
            _summary_

        Args:
            header (_type_): _description_
            data (_type_): _description_
        """
        
        if header == "set_home_gps":
            print(f"set home gps: {data}")
            my_location_alt = self.droneVehicle.location.global_frame
            my_location_alt.lat = data['lat']
            my_location_alt.lon = data['lon']
            my_location_alt.alt = data['alt']
            self.droneVehicle.home_location = my_location_alt
            self.socket_client.send_message("controlMsg", "web", "droneStatus", f"set new home: {data['lat']} - {data['lon']} - {data['alt']}")
        elif header == "run_all_wp":
            self.waypoints = data
            self.set_one_time_home = False
            self.socket_client.send_message("controlMsg", "web", "droneStatus", f"load run all wp with {len(self.waypoints)} wps")
        elif header == "command":
            print(f"command is: {data}")
        elif header == "camera":
            if data == 'on':
                print(f"camera is on")
                self.picam.isRunCamera = True
            elif data == 'off':
                print(f"camera is off")
                self.picam.isRunCamera = False
        elif header == "focusMode":
            if data == 'manual':
                print(f"focusMode is manual")
                self.picam.set_manual_focus(1.0)
            elif data == 'auto':
                print(f"focusMode is auto")
                self.picam.set_continuous_autofocus()
        elif header == "lensFocus":
            self.picam.set_manual_focus(float(data))
            print(f"focusMode is manual with lenFocus: {data}")
        elif header == "zoom":
            self.picam.set_zoom(float(data))
            print(f"zoom is : {data}")
        elif header == "servo":
            if data == 'open':
                self.servo.open_handle()
            elif data == 'close':
                self.servo.close_handle()
        else:
            pass
    
    def handle_drone_event(self, payload):
        """
            _summary_

        Args:
            payload (_type_): _description_
        """
        
        mode = str(self.droneVehicle.mode.name)
        header = payload['header']
        data = payload['data']
        
        print(f"Drone event received with header: {header} and data: {data} and mode: {mode}")
        
        if mode == "GUIDED":
            self.handle_payload_guided(header, data)
        self.handle_payload_otherMode(header, data)
        
        
    def processing_1(self):
        """
            Processing_1 to read camera and stream camera to server
        """
        self.streamer.start_stream()
        while self.running:
            if self.picam.isRunCamera or True:
                self.frame = self.picam.read_camera()
                if self.frame is not None:
                    # self.streamer.process.stdin.write(self.frame.tobytes())
                    self.frame1 = self.frame
                    self.picam.video_writer.write(self.frame)

    def processing_2(self):
        """
            Processing_2 to use detect frame
        """
        while self.running:
            if self.frame is not None:
                # Process frame if necessary
                self.frame, data_qr = self.ob_qr.qr_detect(self.frame)
                self.frame, self.data_objectPerson = self.ob_qr.object_detect(self.frame)
                
                print(f"  data_qr: {data_qr}    -    data_person: {self.data_objectPerson}")
                if time.time() - self.time_send_socket_prev > self.DELAY_SEND_SOCKET:
                    self.time_send_socket_prev = time.time()
                    if len(data_qr) != 0:
                        self.data_qr = data_qr[0]
                        self.socket_client.send_message("controlMsg", "web", "droneFlyStatus", f"QR Detected: {data_qr[0]}")
                    else:
                        self.data_qr = None
                        self.socket_client.send_message("controlMsg", "web", "droneFlyStatus", f"QR none Detected")

    def processing_3(self):
        while self.running:
            if self.frame1 is not None:
                self.streamer.process.stdin.write(self.frame1.tobytes())
                
    def start(self):
        self.thread_1 = threading.Thread(target=self.processing_1, daemon=True)
        self.thread_2 = threading.Thread(target=self.processing_2, daemon=True)
        self.thread_3 = threading.Thread(target=self.processing_3, daemon=True)
        self.thread_1.start()
        self.thread_2.start()
        self.thread_3.start()

    def stop(self):
        self.running = False
        self.thread_1.join()
        self.thread_2.join()
        self.thread_3.join()
        self.streamer.stop_stream()
        self.socket_client.disconnect()
        self.picam.release()
        self.servo.servo_clean()
        self.buttonCoffee.cleanup()
        self.sensorPerson1.cleanup()
        self.sensorPerson2.cleanup()
        self.buzzer.buzzer_clean()

class HandleDelivery:
    def __init__(self) -> None:
        pass
    
    
def main():
    """
        _summary_
    """
    global time_curr, time_prev_send_info, time_prev_action, DELAY_ACTION, DELAY_SEND_INFO

    drone_instance = Drone()
    drone_instance.start()

    try:
        while drone_instance.copter.isConnect:
            time_curr = time.time()
            
            if time_curr - time_prev_send_info > DELAY_SEND_INFO:

                payload_info_drone = drone_instance.copter.read_info_drone()
                drone_instance.socket_client.send_message("controlMsg", "web","droneStatusInfor", payload_info_drone)

                payload_info_atitude = drone_instance.copter.read_info_attitude()
                drone_instance.socket_client.send_message("controlMsg", "web","droneStatusAtitude", payload_info_atitude)

                payload_info_gps = drone_instance.copter.read_info_gps()
                drone_instance.socket_client.send_message("controlMsg", "web","droneStatusGps", payload_info_gps)

                time_prev_send_info = time_curr

            if time_curr - time_prev_action > DELAY_ACTION:
                
                drone_instance.isPerson = drone_instance.sensorPerson1.read_LD2410B() or drone_instance.sensorPerson2.read_LD2410B()
                
                if str(drone_instance.droneVehicle.mode.name) == "GUIDED":
                    if drone_instance.isRunWp is not None and drone_instance.waypoints is not None:
                        if drone_instance.isRunWp is True:
                            if drone_instance.isRunCoffee is True and drone_instance.isRunHome is False:
                                # run to coffee position
                                if drone_instance.waypoint_index < len(drone_instance.waypoints):
                                    drone_instance.current_waypoint = drone_instance.waypoints[drone_instance.waypoint_index]
                                    if drone_instance.waypoint_index_recv != drone_instance.current_waypoint:
                                        drone_instance.flag_change_dis = False
                                        drone_instance.waypoint_index_recv = drone_instance.current_waypoint
                                        drone_instance.copter.goto_waypoint(drone_instance.current_waypoint)
                                    else:
                                        distance = drone_instance.copter.distance_to_current_waypoint(drone_instance.current_waypoint)
                                        print(f"Distance to waypoint: {distance:.2f} meters, waypoint: {drone_instance.waypoint_index}")
                                        if distance < .3:
                                            if drone_instance.flag_change_dis is False:
                                                drone_instance.flag_change_dis = True
                                                print("Waypoint reached!")
                                                if drone_instance.current_waypoint['mode'] == "coffee":
                                                    drone_instance.isFinshedWp = True
                                                else:
                                                    drone_instance.waypoint_index = drone_instance.waypoint_index + 1
                            elif not drone_instance.returnCoffee:
                                # return to home position
                                if not drone_instance.set_one_time_home:
                                    for i in range(len(drone_instance.waypoints)):
                                        if drone_instance.waypoints[i]['mode'] == "home":
                                            drone_instance.waypoint_index_home = i
                                            drone_instance.set_one_time_home = True                        
                                else:
                                    if drone_instance.waypoint_index >= len(drone_instance.waypoints):
                                        drone_instance.waypoint_index = len(drone_instance.waypoints) - 1
                                        
                                    drone_instance.current_waypoint = drone_instance.waypoints[drone_instance.waypoint_index]

                                    if drone_instance.waypoint_index_recv != drone_instance.current_waypoint:
                                        drone_instance.flag_change_dis = False
                                        drone_instance.waypoint_index_recv = drone_instance.current_waypoint
                                        drone_instance.copter.goto_waypoint(drone_instance.current_waypoint)
                                    else:
                                        distance = drone_instance.copter.distance_to_current_waypoint(drone_instance.current_waypoint)
                                        print(f"Distance to waypoint: {distance:.2f} meters, waypoint: {drone_instance.waypoint_index}")
                                        if distance < .3:
                                            if drone_instance.flag_change_dis is False:
                                                drone_instance.flag_change_dis = True
                                                print("Waypoint reached!")
                                                if drone_instance.waypoint_index == drone_instance.waypoint_index_home:
                                                    drone_instance.returnCoffee = True
                                                else:
                                                    drone_instance.waypoint_index = drone_instance.waypoint_index - 1
                                    
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
                                            if drone_instance.data_objectPerson:
                                                drone_instance.buzzer.buzz_pattern([(0.1,0.1)])
                                                drone_instance.droneVehicle.mode = VehicleMode("GUIDED")
                                            else:
                                                drone_instance.droneVehicle.mode = VehicleMode("LAND")
                                            time.sleep(.6)
                                        
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
                                elif drone_instance.isRunHome is False:
                                    if not drone_instance.isPerson:
                                        """
                                            when no person -> arm -> takeoff -> return to home
                                        """
                                        
                                        drone_instance.buzzer.buzz_pattern([(0.1,0.1), (0.3,0.1), (0.1,0.1)])
                                        drone_instance.buzzer.buzz_pattern([(0.1,0.1), (0.3,0.1), (0.1,0.1)])
                                        drone_instance.buzzer.buzz_pattern([(0.1,0.1), (0.3,0.1), (0.1,0.1)])
                                        
                                        time.sleep(2)
                                        
                                        drone_instance.buzzer.buzz_pattern([(0.1,0.1), (0.3,0.1), (0.1,0.1)])
                                        drone_instance.buzzer.buzz_pattern([(0.1,0.1), (0.3,0.1), (0.1,0.1)])
                                        drone_instance.buzzer.buzz_pattern([(0.1,0.1), (0.3,0.1), (0.1,0.1)])
                                        
                                        drone_instance.droneVehicle.armed = True
                                        
                                        while not drone_instance.droneVehicle.armed:
                                            time.sleep(.1)
                                        
                                        aTargetAltitude = 10
                                        
                                        drone_instance.droneVehicle.simple_takeoff(aTargetAltitude)
                                        
                                        while drone_instance.droneVehicle.armed:
                                            current_altitude = drone_instance.copter.vehicle.global_relative_frame.alt
                                            if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
                                                print("Reached target altitude")
                                                break
                                            time.sleep(.2)
                                        
                                        drone_instance.isRunHome = True
                                    else:
                                        drone_instance.buzzer.buzz_pattern([(0.1,0.1)])
                                        time.sleep(.2)
                                        
                                elif drone_instance.returnCoffee:
                                    """
                                        read qr code to handle catch coffee cup
                                    """
                                    if drone_instance.data_qr is not None and drone_instance.data_qr == "1" :
                                        drone_instance.positionCoffee = True
                                    
                                    if drone_instance.positionCoffee or 1:
                                        drone_instance.droneVehicle.mode = VehicleMode("LAND")
                                        
                                        while drone_instance.droneVehicle.armed:
                                            if drone_instance.data_objectPerson:
                                                drone_instance.buzzer.buzz_pattern([(0.1,0.1)])
                                                drone_instance.droneVehicle.mode = VehicleMode("GUIDED")
                                            else:
                                                drone_instance.droneVehicle.mode = VehicleMode("LAND")
                                            time.sleep(.6)
                                        
                                        if not drone_instance.droneVehicle.armed:
                                            drone_instance.droneVehicle.mode = VehicleMode("GUIDED")
                                            
                                        drone_instance.servo.open_handle()
                                
                                        drone_instance.isreturnHome = True
                                        
                                elif drone_instance.isreturnHome:
                                    if not drone_instance.isPerson:
                                        """
                                            when no person -> arm -> takeoff -> return to home
                                        """
                                        
                                        drone_instance.buzzer.buzz_pattern([(0.1,0.1), (0.3,0.1), (0.1,0.1)])
                                        drone_instance.buzzer.buzz_pattern([(0.1,0.1), (0.3,0.1), (0.1,0.1)])
                                        drone_instance.buzzer.buzz_pattern([(0.1,0.1), (0.3,0.1), (0.1,0.1)])
                                        
                                        time.sleep(2)
                                        
                                        drone_instance.buzzer.buzz_pattern([(0.1,0.1), (0.3,0.1), (0.1,0.1)])
                                        drone_instance.buzzer.buzz_pattern([(0.1,0.1), (0.3,0.1), (0.1,0.1)])
                                        drone_instance.buzzer.buzz_pattern([(0.1,0.1), (0.3,0.1), (0.1,0.1)])
                                        
                                        drone_instance.droneVehicle.armed = True
                                        
                                        while not drone_instance.droneVehicle.armed:
                                            time.sleep(.1)
                                        
                                        aTargetAltitude = 10
                                        
                                        drone_instance.droneVehicle.simple_takeoff(aTargetAltitude)
                                        
                                        while drone_instance.droneVehicle.armed:
                                            current_altitude = drone_instance.copter.vehicle.global_relative_frame.alt
                                            if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
                                                print("Reached target altitude")
                                                break
                                            time.sleep(.2)
                                        
                                        drone_instance.droneVehicle.mode = VehicleMode("RTL")
                                    else:
                                        drone_instance.buzzer.buzz_pattern([(0.1,0.1)])
                                        time.sleep(.2)
                time_prev_action = time_curr

    except:
        drone_instance.stop()

if __name__ == "__main__":
    main()
