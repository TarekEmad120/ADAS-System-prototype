import carla
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
class RadarSensor(object):

    def __init__(self, parent_actor, hud):
        '''
        * Service Name: __RadarSensor__
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): parent_actor, hud
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: to initialize the radar sensor and set its attributes
        '''
        self.sensors = []
        self._parent = parent_actor
        self.hud = hud
        self.velocity_range = 7.5  # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', '45')  
        bp.set_attribute('vertical_fov', '5')
        bp.set_attribute('range', '10')

        self.radar_positions = [
            (carla.Transform(carla.Location(x=-1.5, y=-0.7, z=1.5), carla.Rotation(yaw=180)), 'Left Mirror'),
            (carla.Transform(carla.Location(x=-1.5, y=0.7, z=1.5), carla.Rotation(yaw=180)), 'Right Mirror')
        ]
        
        for transform, position in self.radar_positions:
            sensor = world.spawn_actor(bp, transform, attach_to=self._parent)
            sensor.listen(lambda radar_data, pos=position: self._Radar_callback(radar_data, pos))
            self.sensors.append(sensor)


    def _Radar_callback(self, radar_data, position):
        '''
        * Service Name: __Radar_callback__
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): radar_data, position
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: to process the radar data and check for vehicles approaching the car by calculating its relative velocity
        *              with respect to the car's speed and checking if the object is moving towards the carusing depth and velocity
        *              and if the object is within the radar's field of view then it will alert the user usnig async_buzzer()
        '''
        car_velocity = self._parent.get_velocity()
        car_speed = math.sqrt(car_velocity.x**2 + car_velocity.y**2 + car_velocity.z**2)
        
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            if abs(azi) <= 30 and detect.depth > 1.0: 
                relative_velocity = detect.velocity - car_speed
                if relative_velocity < -1.9:  
                    self.hud.notification(f'Vehicle approaching from {position}!')
                    print(f'Vehicle approaching from {position}: Depth {detect.depth:.2f} m, Velocity {relative_velocity:.2f} m/s')
                    # async_buzzer()
                    current_rot = radar_data.transform.rotation
                    fw_vec = carla.Vector3D(x=detect.depth - 0.25)
                    world_transform = carla.Transform(
                        carla.Location(),
                        carla.Rotation(
                            pitch=current_rot.pitch + math.degrees(detect.altitude),
                            yaw=current_rot.yaw + azi,
                            roll=current_rot.roll))
                    point_location = radar_data.transform.location + world_transform.transform(fw_vec)

    def destroy(self):
        for sensor in self.sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()