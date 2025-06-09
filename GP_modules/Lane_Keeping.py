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


def _create_singleton_lka_thread(self, deviation, world_object):
    '''
    * Service Name: _create_singleton_lka_thread
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): deviation, world_object
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to create a singleton LKA thread for lane keeping assist
    *              and apply lane keeping assist correction
    '''
    if not getattr(world_object, 'lka_enabled', True):
        print("LKA is disabled, not creating LKA thread")
        return

    if hasattr(self, 'lka_thread') and self.lka_thread is not None and self.lka_thread.is_alive():
        print("LKA thread already running, not creating a new one")
        return
    
    world_object.lka_active = True
    world_object.lka_last_correction_time = time.time()
    world_object.lka_deviation = deviation
    world_object.lka_continuous_thread_active = True
    
    print("Starting new LKA correction thread...")
    self.lka_thread = threading.Thread(
        target=self._continuous_lane_correction,
        args=(world_object,),
        name="LKA_Thread",
        daemon=True
    )
    self.lka_thread.start()
    if self.lka_thread.is_alive():
        print("LKA thread successfully started!")
    else:
        print("LKA thread failed to start!")
        world_object.lka_continuous_thread_active = False

'''
* Service Name: _continuous_lane_correction
* Sync/Async: Synchronous
* Reentrancy: Non-Reentrant
* Parameters (in): world_object
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: to apply lane keeping assist correction continuously
*              and adjust the steering based on lane deviation
'''
def _continuous_lane_correction(self, world_object):

    print("Lane keeping assist thread running")
    world_object.lka_continuous_thread_active = True
    min_threshold = 5.0      
    center_threshold = 15.0   
    max_dev = 100.0     
    damp_factor = 0.7      
    center_count = 0          
    center_steps = 3
    max_steer_per_step = 0.03 
    max_steer = 0.7
    last_steer = 0.0
    last_speed = 0.0
    last_dev = 0.0
    target_steer = 0.0
    '''
    the idea here that we created thread like microcontroller
    that will run continuously and check the lane deviation and try to keep the car in lane 
    and if the car is centered in the lane for a certain number of frames
    it depends on the speed of the car and the deviation as PD controller
    and if the car is not centered it will try to correct the steering
    '''
    try:
        '''
        initailly after checking the world object
        we will check if the lane keeping assist is enabled and active
        and if it is we will start the lane correction thread
        and we will run the lane correction thread until the car is centered in the lane
        '''
        while (world_object.lka_continuous_thread_active and hasattr(world_object, 'lka_active') and 
            world_object.lka_active and 
            world_object.lka_enabled):
            if not world_object.lka_enabled:
                print("LKA disabled during correction - thread exiting")
                break
            with self.lane_state_lock:
                is_centered = self.crossing_state.get('center', False)
                current_deviation = self.deviation

            velocity = self._parent.get_velocity()
            car_speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) 


            '''
            after that we need to check if the car is centered in the lane by 
            calculating adaptive center threshold based on the car speed
            and adaptive damping factor based on the car speed
            and if the car is centered we will stop the lane correction thread
            '''
            adaptive_center_threshold = center_threshold * (1 + (car_speed / 100.0))
            adaptive_damping = damp_factor * (1 + (car_speed / 150.0))
            in_center_zone = abs(current_deviation) < adaptive_center_threshold
            if in_center_zone:
                center_count += 1
                if center_count >= center_steps:
                    world_object.lka_active = False
                    print(f"Car centered (deviation={current_deviation:.1f}px < threshold={adaptive_center_threshold:.1f}px) - LKA deactivated")
                    break
            else:
                center_count = 0
            if abs(current_deviation) < min_threshold:
                time.sleep(0.05)
                continue
            current_control = self._parent.get_control()
            current_steer = current_control.steer


            '''
            then we need to calculate the steering correction based on the current deviation  
            and the maximum deviation and the car speed in km/h 
            and apply the correction to the steering
            and if the correction is applied we will update the last steer and last deviation
            and last speed. this will help us to smooth the steering correction
            and we will print the deviation, speed and applied steering correction
            values of correction based on trial and error
            '''

            norm_deviation = -current_deviation / max_dev
            norm_deviation = max(-1.0, min(1.0, norm_deviation))
            if abs(norm_deviation) > 0.5:
                correction_factor = 0.3
            else:
                correction_factor = 0.25
            if car_speed > 60:
                correction_factor *= 0.7
            elif car_speed > 40:
                correction_factor *= 0.85
            deviation_change = current_deviation - last_dev
            derivative_factor = -deviation_change * 0.0005
            raw_correction = norm_deviation * correction_factor + derivative_factor
            target_steer = current_steer + raw_correction
            steer_change = target_steer - current_steer
            clamped_steer_change = max(-max_steer_per_step, min(max_steer_per_step, steer_change))
            smoothed_steer_change = clamped_steer_change * (1.0 - adaptive_damping)
            final_steer = current_steer + smoothed_steer_change
            final_steer = max(-max_steer, min(max_steer, final_steer))
            if abs(final_steer - current_steer) > 0.001:
                control = carla.VehicleControl()
                control.throttle = current_control.throttle
                control.brake = current_control.brake
                control.hand_brake = current_control.hand_brake
                control.reverse = current_control.reverse
                control.manual_gear_shift = current_control.manual_gear_shift
                control.gear = current_control.gear
                control.steer = final_steer
                self._parent.apply_control(control)
                last_steer = final_steer
            last_dev = current_deviation
            last_speed = car_speed
            print(f"LKA: deviation={current_deviation:.1f}px, speed={car_speed:.1f}km/h, applied={final_steer:.3f}")
            time.sleep(0.05)
                
    except Exception as e:
        print(f"Error in lane correction thread: {e}")
    finally:
        if hasattr(world_object, 'lka_continuous_thread_active'):
            world_object.lka_continuous_thread_active = False
        print("Lane keeping assist thread stopped")
