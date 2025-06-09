import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import gc
import weakref
import struct
import os
import sys
import queue
import winsound
import threading
import time
import cv2
import numpy as np
import pygame


class EmergencyBrakingSystem(object):
    '''
    * Service Name: __EmergencyBrakingSystem__
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): parent_actor, hud, max_distance
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to initialize the emergency braking system and set its attributes
    *              Creating the radar sensors and set their attributes
    *              and start the safety thread for continuous braking with setting multiple stages
    '''
    def __init__(self, parent_actor, hud, max_distance=0.0):
        self._parent = parent_actor
        self.hud = hud
        self.max_distance = max_distance
        self.last_brake_time = 0
        self.brake_active = False
        self.debug = False 
        self.override_active = False 
        
        '''------------multi-stage parameters-----------'''
        self.warn_level = 0     
        self.brake_level = 0   
        self.collision_risk = 0  
        
        '''--------Advanced filtering parameters--------'''
        self.detection_counter = 0     
        self.detection_threshold = 5  
        self.warn_counter = 0         
        self.warn_threshold = 2       
        self.false_detection_rate = 2

        '''----------Driver override parameters----------'''
        self.last_intervention_time = 0
        # the time needed to wait before re-enabling the system for the driver to take control again
        self.cooling_period = 3.0     
        self.driver_override = False  
        self.last_warning_time = 0  

        self.sensors = []
        world = self._parent.get_world()
        self._create_radar_sensors(world)
        
        '''------safety thread parameters------'''
        self.safety_thread_active = True
        self.safety_thread = threading.Thread(target=self._safety_thread_function)
        self.safety_thread.daemon = True
        self.safety_thread.start()
        

    def _create_radar_sensors(self, world):
        '''
        * Service Name: _create_radar_sensors_
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): world
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: to create the radar sensors and set their attributes then attach them to the vehicle
        *             there is only 3 radars in front of car to simulate real world configurations in Automotive industry
        '''
        #center radar 
        front_bp = world.get_blueprint_library().find('sensor.other.radar')
        front_bp.set_attribute('horizontal_fov', '20')
        front_bp.set_attribute('vertical_fov', '5')
        front_bp.set_attribute('range', str(self.max_distance))
        front_transform = carla.Transform(carla.Location(x=2.8, z=0.8), carla.Rotation(yaw=0))
        front_sensor = world.spawn_actor(front_bp, front_transform, attach_to=self._parent)
        weak_self = weakref.ref(self)
        front_sensor.listen(lambda radar_data: EmergencyBrakingSystem._safe_radar_callback(weak_self, radar_data, "front"))
        self.sensors.append(front_sensor)
        
        # Left-front radar 
        left_bp = world.get_blueprint_library().find('sensor.other.radar')
        left_bp.set_attribute('horizontal_fov', '15')
        left_bp.set_attribute('vertical_fov', '5')
        left_bp.set_attribute('range', str(self.max_distance))
        left_transform = carla.Transform(carla.Location(x=2.3, y=-0.7, z=0.8), carla.Rotation(yaw=-25))
        left_sensor = world.spawn_actor(left_bp, left_transform, attach_to=self._parent)
        left_sensor.listen(lambda radar_data: EmergencyBrakingSystem._safe_radar_callback(weak_self, radar_data, "left"))
        self.sensors.append(left_sensor)
        
        # Right-front radar
        right_bp = world.get_blueprint_library().find('sensor.other.radar')
        right_bp.set_attribute('horizontal_fov', '15')
        right_bp.set_attribute('vertical_fov', '5')
        right_bp.set_attribute('range', str(self.max_distance))
        right_transform = carla.Transform(carla.Location(x=2.3, y=0.7, z=0.8), carla.Rotation(yaw=25))
        right_sensor = world.spawn_actor(right_bp, right_transform, attach_to=self._parent)
        right_sensor.listen(lambda radar_data: EmergencyBrakingSystem._safe_radar_callback(weak_self, radar_data, "right"))
        self.sensors.append(right_sensor)


    @staticmethod
    def _safe_radar_callback(weak_self, radar_data, sensor_position):
        '''
        * Service Name: _safe_radar_callback_
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): weak_self, radar_data, sensor_position
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: to safely call the radar callback function using weak reference to avoid circular references
        '''
        self = weak_self()
        if not self:
            return  
        try:
            if not self._parent or not self._parent.is_alive:
                return 
            self._radar_callback(radar_data, sensor_position)
        except RuntimeError as e:
            if "destroyed actor" in str(e):
                return
            else:
                print(f"Error in radar callback: {e}")
        

    def _radar_callback(self, radar_data, sensor_position):
        '''
        * Service Name: _radar_callback_
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): radar_data, sensor_position
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: to process the radar data and check for objects in the vicinity of the vehicle
        *              and filter the readings to reduce noise and stabilize the measurements to help it 
        *             detect objects more accurately and verify it in harsh conditions then send the data to TivaC using
        *              UART serial communication
        '''
        min_distance = self.max_distance
        rel_velocity = 0.0
        ttc = float('inf')
        car_velocity = self._parent.get_velocity()
        car_speed = math.sqrt(car_velocity.x**2 + car_velocity.y**2 + car_velocity.z**2)
        detected = False
        world = self._parent.get_world()
        if hasattr(world, 'ultrasonic_sensors') and hasattr(world.ultrasonic_sensors, 'auto_parking_active'):
            if world.ultrasonic_sensors.auto_parking_active:
                self.warn_level = 0
                self.brake_level = 0
                self.override_active = False
                self.detection_counter = 0
                self.warn_counter = 0
                self.collision_risk = 0
                return

        
        # Skip processing at very low speeds unless very close objects
        if car_speed < 2.0 and not self.brake_active:
            self.warn_level = 0
            self.brake_level = 0
            return
        
        # Get current control to check driver inputs
        current_control = self._parent.get_control()
        steering_input = current_control.steer
        throttle_input = current_control.throttle
        brake_input = current_control.brake
        
        # Check if driver is actively trying to override or still in control
        driver_override_intent = (abs(steering_input) > 0.3) or (throttle_input > 0.7) or (brake_input > 0.7)
        if driver_override_intent and self.warn_level > 0:
            self.driver_override = True
            self.warn_counter = 0
            self.detection_counter = 0
            if self.warn_level > 1:
                # print("[EBS] Driver override detected - reducing intervention")
                self.warn_level = min(1, self.warn_level) 
                self.brake_level = 0
            return
        else:
            self.driver_override = False

        max_azimuth = 2 if car_speed > 15 else (3 if sensor_position == "front" else 4)
    
        # now we will calculate the width of the vehicle with a safety margin to keep it safeand avoid accidents
        safety_margin = 0.2 if car_speed < 10 else 0.5
        vehicle_width = self._parent.bounding_box.extent.y * 2 + safety_margin
        
        # Calculate predicted vehicle path based on steering angle
        # This is a simplification of where the car will be in the next second
        steering_angle_rad = steering_input * 0.5 
        predicted_direction = math.tan(steering_angle_rad) * 5.0 
        corridor_width_factor = min(1.0, car_speed / 10.0) + 0.3  
        potential_vehicle_detections = []
        potential_stationary_detections = []
        
        '''
        here we will iterate over the radar data and check for objects in the vicinity of the vehicle
        then calculate the azimuth and altitude of the object with calculating the lateral distance
        the lateral distance is the distance between the object and the vehicle in the lateral direction
        then we will check if the object is within the dynamic FOV that narrows with speed
        then if the object is above ground level then checks object is in the actual vehicle's projected path considering steering
        after that we will classify the object as a vehicle or stationary object then we will check if the object is within the minimum distance
        '''
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = detect.altitude
            alt_deg = math.degrees(alt)
            lateral_distance = math.sin(math.radians(azi)) * detect.depth
            adjusted_lateral = lateral_distance - (predicted_direction * detect.depth / 5.0)
            if alt < -0.15:
                continue
                
            # Only consider obstacles that are:
            # 1. Within a dynamic FOV that narrows with speed
            # 2. Above ground level
            # 3. In the actual vehicle's projected path considering steering
            # 4. Within the width corridor adjusted for speed
            corridor_half_width = (vehicle_width/2) * corridor_width_factor
            
            if (abs(azi) < max_azimuth and                          
                alt > -0.15 and                                       
                abs(adjusted_lateral) < corridor_half_width and detect.depth > 0.5):                                 
                if abs(detect.velocity) > 0.5:
                    potential_vehicle_detections.append((detect, adjusted_lateral))
                else:
                    potential_stationary_detections.append((detect, adjusted_lateral))
          
                if detect.depth < min_distance:
                    min_distance = detect.depth
                    rel_velocity = car_speed - detect.velocity
                    detected = True
                
                if self.debug and detect.depth < 30: 
                    print(f"[EBS-{sensor_position}] Radar: depth={detect.depth:.1f}m, azi={azi:.1f}°, lat={adjusted_lateral:.1f}m, vel={detect.velocity:.1f}m/s")

        if detected and rel_velocity > 0.1:
            ttc = min_distance / rel_velocity
            self.collision_risk = self._calculate_collision_risk(ttc, min_distance, rel_velocity, car_speed)
            if self.debug and detected:
                print(f"[EBS] min_distance={min_distance:.1f}m, rel_velocity={rel_velocity:.1f}m/s, TTC={ttc:.1f}s, risk={self.collision_risk}/10")
        
        if car_speed < 5.0:
            # the visual warning is a visual warning (flashing light) to warn the driver.
            visual_warning_ttc = 3.0
            # the acoustic warning plays an acoustic (sound) warning.
            acoustic_warning_ttc = 2.0
            # the light brake is a light brake (30% brake) to warn the driver.
            light_brake_ttc = 1.5
            # the medium brake is a medium brake (60% brake) to warn the driver.
            medium_brake_ttc = 1.0
            # the full brake is a full brake (100% brake) to warn the driver.
            full_brake_ttc = 0.7
            visual_warning_dist = 6.0
            acoustic_warning_dist = 4.0
            light_brake_dist = 3.0
            medium_brake_dist = 2.0
            full_brake_dist = 1.2

        # Urban arterials    
        elif car_speed < 15.0:  
            visual_warning_ttc = 3.0
            acoustic_warning_ttc = 2.0
            light_brake_ttc = 1.8
            medium_brake_ttc = 1.2
            full_brake_ttc = 0.8
            visual_warning_dist = 12.0
            acoustic_warning_dist = 8.0
            light_brake_dist = 5.0
            medium_brake_dist = 3.5
            full_brake_dist = 2.0

        # Highway speeds    
        else:  
            visual_warning_ttc = 3.5
            acoustic_warning_ttc = 2.5
            light_brake_ttc = 2.0
            medium_brake_ttc = 1.5
            full_brake_ttc = 1.0
            
            visual_warning_dist = 20.0
            acoustic_warning_dist = 15.0
            light_brake_dist = 12.0
            medium_brake_dist = 8.0
            full_brake_dist = 5.0
            

        reverse_gear = current_control.reverse
        if detected and not reverse_gear:
            self.warn_counter += 1
            # Visual warning
            if (ttc < visual_warning_ttc or min_distance < visual_warning_dist) and self.warn_counter >= self.warn_threshold:
                if self.warn_level < 1 and time.time() - self.last_warning_time > 3.0:
                    self.hud.notification("! Obstacle Ahead !", seconds=1.0)
                    self.last_warning_time = time.time()
                self.warn_level = max(self.warn_level, 1)  
                
            # Acoustic warning
            if (ttc < acoustic_warning_ttc or min_distance < acoustic_warning_dist) and self.warn_counter >= self.warn_threshold:
                if self.warn_level < 2:
                    async_buzzer()
                self.warn_level = max(self.warn_level, 2) 
            # Light brake
            if (ttc < light_brake_ttc or min_distance < light_brake_dist) and self.detection_counter >= self.detection_threshold:
                self.brake_level = max(self.brake_level, 1) 
            # Medium brake
            if (ttc < medium_brake_ttc or min_distance < medium_brake_dist) and self.detection_counter >= self.detection_threshold:
                self.brake_level = max(self.brake_level, 2) 
            # Full brake 
            if (ttc < full_brake_ttc or min_distance < full_brake_dist) and self.detection_counter >= self.detection_threshold:
                self.brake_level = 3 
            self.detection_counter = min(self.detection_counter + 1, self.detection_threshold * 2)
            self._apply_staged_braking()

        else:
            self.detection_counter = max(0, self.detection_counter - 1)
            self.warn_counter = max(0, self.warn_counter - 1)
            
            if self.detection_counter == 0:
                self.warn_level = 0
                self.brake_level = 0
                self.override_active = False
                self.collision_risk = 0

    def _calculate_collision_risk(self, ttc, distance, rel_velocity, speed):
        '''
        * Service Name: _calculate_collision_risk_
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): ttc, distance, rel_velocity, speed
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: risk
        * Description: to calculate the collision risk based on time-to-collision (TTC), distance, relative velocity, and speed
        *              and return a risk value between 0 and 10
        '''
        if ttc > 3.0:
            ttc_risk = 0
        elif ttc > 2.0:
            ttc_risk = 3
        elif ttc > 1.5:
            ttc_risk = 5
        elif ttc > 1.0:
            ttc_risk = 7
        else:
            ttc_risk = 10

        distance_factor = min(10, 20 / (distance + 0.1))
        speed_factor = min(10, speed / 5) 
        risk = (ttc_risk * 0.5) + (distance_factor * 0.3) + (speed_factor * 0.2)
        return min(10, risk)


    def _apply_staged_braking(self):
        '''
        * Service Name: _apply_staged_braking_
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): None
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: to apply staged braking based on the current brake level
        *              and check if the driver override is active or not
        '''    
        if self.driver_override:
            return
        if self.detection_counter < self.detection_threshold:
            return
        if self.brake_level <= 1 and time.time() - self.last_intervention_time < self.cooling_period:
            return
        current_control = self._parent.get_control()
        
        # conditions of applying appropriate brake force according to level (1 to 3)
        # Light braking (30%)
        if self.brake_level == 1:  
            control = current_control
            control.throttle = 0.0
            control.brake = 0.3
            control.hand_brake = False
            self._parent.apply_control(control)
            time.sleep(1.0)  # Hold light brake for 1 second
            if not self.brake_active:
                self.hud.notification("Brake Assist", seconds=1.0)
            self.brake_active = True
            self.override_active = False

        # Medium braking (60%)     
        elif self.brake_level == 2: 
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 0.6
            control.hand_brake = False
            control.steer = current_control.steer * 0.5  
            control.manual_gear_shift = False
            self._parent.apply_control(control)
            time.sleep(1.0)

            if not self.brake_active or self.brake_level == 1:
                self.hud.notification("BRAKE ASSIST", seconds=1.5)
                async_buzzer()
            self.brake_active = True
            self.override_active = True  
        
        # Full braking (100%)
        elif self.brake_level == 3:
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 1.0
            control.hand_brake = True
            control.manual_gear_shift = True
            control.gear = 0  # Neutral
            self._parent.apply_control(control)
            time.sleep(1.0) 
            if not self.brake_active or self.brake_level < 3:
                self.hud.notification("EMERGENCY BRAKE!", seconds=2.0)
                async_buzzer()

            self.brake_active = True
            self.override_active = True  
        self.last_intervention_time = time.time()
    


    def _safety_thread_function(self):
        '''
        * Service Name: _safety_thread_function_
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): None
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: to run the safety thread that continuously applies braking during emergencies
        *              and check the brake level and override status
        '''
        while self.safety_thread_active:
            try:
                if self.brake_level == 3 and self.override_active:
                    velocity = self._parent.get_velocity()
                    speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                    if speed_kmh > 0.5:
                        control = carla.VehicleControl()
                        control.throttle = 0.0
                        control.brake = 1.0
                        control.manual_gear_shift = True
                        control.reverse = False
                        control.hand_brake = True
                        control.gear = 0 
                        self._parent.apply_control(control)
                        if self.debug:
                            print("[EBS] Emergency brake applied: level 3, speed {:.2f} km/h".format(speed_kmh))
                    else:
                        control = carla.VehicleControl()
                        control.throttle = 0.0
                        control.brake = 1.0
                        control.manual_gear_shift = True
                        control.reverse = False
                        control.hand_brake = True
                        control.gear = 0
                        self._parent.apply_control(control)
                elif self.brake_level == 2 and self.override_active:
                    control = carla.VehicleControl()
                    control.throttle = 0.0
                    control.brake = 0.6
                    control.manual_gear_shift = True
                    control.reverse = False
                    control.hand_brake = False
                    control.gear = 0
                    self._parent.apply_control(control)
                    self.brake_level = 0
                    self.override_active = False
                else:
                    pass
                time.sleep(0.01) 
            except Exception as e:
                print(f"[EBS] Safety thread error: {e}")
                time.sleep(0.5)


    def destroy(self):
        '''
        * Service Name: __destroy__
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): None
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: to destroy the emergency braking system and stop the safety thread
        *              and clean up all sensors
        '''
        if hasattr(self, 'safety_thread_active'):
            self.safety_thread_active = False
        if hasattr(self, 'safety_thread') and self.safety_thread is not None:
            self.safety_thread.join(timeout=1.0)

        for sensor in self.sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        self.sensors = []