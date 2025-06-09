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

def _lane_departure_warning_thread(self):
    '''
    * Service Name: _lane_departure_warning_thread
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): None
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to run the lane departure warning thread and check the lane crossing state
    '''
    print("Lane departure warning thread running")
    crossing_persistence = {'right': 0, 'left': 0, 'center': 0}
    threshold = 0 
    margin = 0     
    persistence_threshold = 2
    warning_cooldown = 3.0
    last_warning_time = 0.0
    self.lka_thread = None
    world = None
    try:
        world = self._parent.get_world()
        if not hasattr(world, 'lka_enabled'):
            if lka_active_confirm is True:
                world.lka_enabled = True
            else:
                world.lka_enabled = False
        print(f"LKA initially enabled: {world.lka_enabled}")
    except Exception as e:
        print(f"Error initializing LKA: {e}")
    
    while self.ldw_thread_active:
        try:
            try:
                lane_data = self.lane_data_queue.get(block=True, timeout=0.1)
            except queue.Empty:
                time.sleep(0.01)
                continue
            try:
                world = self._parent.get_world()
                if not hasattr(world, 'lka_enabled'):
                    if lka_active_confirm is True:
                        world.lka_enabled = True
                    else:
                        world.lka_enabled = False
            except Exception as e:
                print(f"Error accessing world: {e}")
                time.sleep(0.2)
                continue
                
            deviation = lane_data['deviation']
            width = lane_data['width']
            threshold = width // 40
            margin = threshold // 4
            if random.random() < 0.01:
                lka_status = f"LKA enabled: {world.lka_enabled}, active: {getattr(world, 'lka_active', False)}"
                print(lka_status)
            if deviation > threshold + margin:
                crossing_persistence['right'] += 1
                crossing_persistence['left'] = 0
                crossing_persistence['center'] = 0
            elif deviation < -threshold - margin:
                crossing_persistence['left'] += 1
                crossing_persistence['right'] = 0
                crossing_persistence['center'] = 0
            else:
                crossing_persistence['center'] += 1
                crossing_persistence['right'] = 0
                crossing_persistence['left'] = 0
            
            '''---- LANE CROSSING DETECTION ----'''
            current_time = time.time()
            vehicle_lights = self._parent.get_light_state()
            right_blinker_on = bool(vehicle_lights & carla.VehicleLightState.RightBlinker)
            left_blinker_on = bool(vehicle_lights & carla.VehicleLightState.LeftBlinker)
            if crossing_persistence['right'] >= persistence_threshold:
                new_crossing = {"right": True, "left": False, "center": False}
                with self.lane_state_lock:
                    self.crossing_state = new_crossing
                    self.deviation = deviation
                if not right_blinker_on and current_time - last_warning_time > warning_cooldown:
                    self.hud.notification("Lane Departure Warning - Use Turn Signal", seconds=2.0)
                    async_buzzer()
                    last_warning_time = current_time
                    print("Right lane departure without blinker detected")
                # Check if LKA should be activated
                if world.lka_enabled:
                    thread_active = hasattr(self, 'lka_thread') and self.lka_thread is not None and self.lka_thread.is_alive()
                    if not thread_active:
                        print(f"ACTIVATING LKA for right departure (deviation={deviation}px)")
                        world.lka_active = True
                        self._create_singleton_lka_thread(deviation, world)
                    else:
                        with self.lane_state_lock:
                            self.deviation = deviation
                
            elif crossing_persistence['left'] >= persistence_threshold:
                new_crossing = {"right": False, "left": True, "center": False}
                with self.lane_state_lock:
                    self.crossing_state = new_crossing
                    self.deviation = deviation
                if not left_blinker_on and current_time - last_warning_time > warning_cooldown:
                    self.hud.notification("Lane Departure Warning - Use Turn Signal", seconds=2.0)
                    async_buzzer()
                    last_warning_time = current_time
                    print("Left lane departure without blinker detected")
                if world.lka_enabled:
                    thread_active = hasattr(self, 'lka_thread') and self.lka_thread is not None and self.lka_thread.is_alive()
                    if not thread_active:
                        print(f"ACTIVATING LKA for left departure (deviation={deviation}px)")
                        world.lka_active = True
                        self._create_singleton_lka_thread(deviation, world)
                    else:
                        with self.lane_state_lock:
                            self.deviation = deviation
    
            elif crossing_persistence['center'] >= persistence_threshold:
                new_crossing = {"right": False, "left": False, "center": True}
                with self.lane_state_lock:
                    self.crossing_state = new_crossing
                    self.deviation = 0
                if hasattr(world, 'lka_active') and world.lka_active:
                    world.lka_active = False

            time.sleep(0.01)
        except Exception as e:
            print(f"Error in lane departure warning thread: {e}")
            time.sleep(0.1) 