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


class AutoParking(object):
    '''----- Auto parking safety parameters -----'''
    MIN_PARKING_SIZE_SPOT = 300
    SAFE_DISTANCE = 100
    DANGER_DISTANCE = 10
    ALLIGNMENT_THRESHOLD = 10
    ALLIGNMENT_DISTANCE = 60
    
    '''----- Auto parking states -----'''
    PARKING_STATE_IDLE = 0
    PARKING_STATE_FORWARD = 1
    PARKING_STATE_BACKWARD = 2
    PARKING_STATE_RIGHT = 3
    PARKING_STATE_LEFT = 4
    PARKING_STATE_ALLIGN = 5
    PARKING_STATE_STOP = 6
    PARKING_STATE_SEARCHING = 7
    PARKING_STATE_FINISHED = 8
    PARKING_STATE_PERP_SEARCHING = 9
    PARKING_STATE_PERP_POSITIONING = 10
    PARKING_STATE_PERP_MANEUVERING = 11
    PARKING_STATE_PERP_ADJUSTING = 12

    '''----- Auto parking control commands -----'''
    CONTROL_FORWARD = 1
    CONTROL_STOP = 2
    CONTROL_STEER_LEFT = 3
    CONTROL_STEER_RIGHT = 4
    CONTROL_FORWARD_LEFT = 5
    CONTROL_FORWARD_RIGHT = 6
    CONTROL_REVERSE_LEFT = 7
    CONTROL_REVERSE_RIGHT = 8
    CONTROL_BACKWARD = 9
    CONTROL_FORWARD_RIGHT_PREP = 10
    CONTROL_FORWARD_LEFT_PREP = 11
    '''----- Auto parking state variables -----'''
    autoparkstate = False
    autoparkstate2 = False

    def __init__(self, parent_actor, hud, ultrasonic_system):
        '''
        * Service Name: AutoParking
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): parent_actor, hud, ultrasonic_system
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: this class implements an auto-parking system that uses ultrasonic sensors to detect parking spots 
                        and control the vehicle's movement.
        '''
        self.parent_actor = parent_actor
        self.hud = hud
        self.ultrasonic_system = ultrasonic_system
        self.parking_status = self.PARKING_STATE_IDLE
        self.parking_active = False
        self.debug_mode = True 
        self.simulation_time = 0.0
        self.parking_mode = "none"

    def init(self):
        '''
        * Service Name: init
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): None
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: Initializes the auto parking system, setting the initial state to idle and notifying the HUD.
        '''
        self.parking_status = self.PARKING_STATE_IDLE
        self.parking_active = False
        self.hud.notification("Auto-parking system initialized")

    def activate_parallel(self):
        '''
        * Service Name: activate_parallel
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): None
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: Activates the parallel auto parking system, setting the state to idle and notifying the HUD.
        '''
        self.parking_status = self.PARKING_STATE_IDLE
        self.parking_active = True
        self.hud.notification("Auto-parking activated")
        

    def activate(self):
        '''
        * Service Name: activate
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): None
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: Activates the auto parking system, which is currently set to parallel parking mode.
        '''
        self.activate_parallel()


    def activate_perpendicular(self):
        '''
        * Service Name: activate_perpendicular
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): None
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: Activates the perpendicular auto parking system, 
                    setting the state to prependicular searching and notifying the HUD.
        '''
        self.parking_status = self.PARKING_STATE_PERP_SEARCHING
        self.parking_active = True
        self.parking_mode = "perpendicular"
        self.hud.notification("Perpendicular auto-parking activated")
        print("Starting perpendicular parking search...")
        if hasattr(self, 'perp_phase'):
            self.perp_phase = 0
    

    def deactivate(self):
        '''
        * Service Name: deactivate
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): None
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: Deactivates the auto parking system, 
                    setting the state to idle and notifying the HUD.
        '''
        self.parking_status = self.PARKING_STATE_IDLE
        self.parking_active = False
        self.send_data(self.CONTROL_STOP, 0)
        self.hud.notification("Auto-parking deactivated")
    

    def is_active(self):
        '''
        * Service Name: is_active
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): None
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: bool
        * Description: Checks if the auto parking system is currently active.
        '''
        return self.parking_active
    

    def get_state(self):
        '''
        * Service Name: get_state
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): None
        * Parameters (inout): None
        * Parameters (out): parking_status
        * Return value: int
        * Description: Returns the current state of the auto parking system.
        '''
        return self.parking_status
    

    def process(self, sensor_readings, vehicle_speed):
        '''
        * Service Name: process
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): sensor_readings, vehicle_speed
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: Processes sensor readings and vehicle speed to control the parking system.
        * It implements a state machine to manage the parking process, including searching for a parking spot,
        * aligning the vehicle, and maneuvering into the spot using S curve maneuvers.
        '''
        if not self.parking_active:
            return
        self.simulation_time = self.parent_actor.get_world().get_snapshot().timestamp.elapsed_seconds
        frontDistance1 = sensor_readings[0]
        frontDistance2 = sensor_readings[1]
        frontFarLeftDistance = sensor_readings[2]
        frontFarRightDistance = sensor_readings[3]

        rearDistance1 = sensor_readings[4]
        rearDistance2 = sensor_readings[5]
        rearFarLeftDistance = sensor_readings[6]
        rearFarRightDistance = sensor_readings[7]

        leftFrontDistance = sensor_readings[8]
        leftRearDistance = sensor_readings[9]

        rightFrontDistance = sensor_readings[10]
        rightRearDistance = sensor_readings[11]
        if hasattr(self, 'parking_mode') and self.parking_mode == "perpendicular":
            self.process_perpendicular_parking(sensor_readings, vehicle_speed)
        else:
            if self.debug_mode:
                debug_msg = f"Front: {frontDistance1}, {frontDistance2}, Front Far L/R: {frontFarLeftDistance}, {frontFarRightDistance}"
                print(debug_msg)
                debug_msg = f"Rear: {rearDistance1}, {rearDistance2}, Rear Far L/R: {rearFarLeftDistance}, {rearFarRightDistance}"
                print(debug_msg)
                debug_msg = f"Side: Left F/R: {leftFrontDistance}, {leftRearDistance}, Right F/R: {rightFrontDistance}, {rightRearDistance}"
                print(debug_msg)


            # the first sate is always idle
            if self.parking_status == self.PARKING_STATE_IDLE:
                self.parking_status = self.PARKING_STATE_SEARCHING
                print("State: SEARCHING")
                self.send_data(self.CONTROL_FORWARD, 20)
                self.autoparkstate = False
                self.autoparkstate2 = False
            # Second state is searching for a parking spot by moving forward
            # to check if there is a parking spot and if the parking spot is big enough
            # the car will move past the parking spot and then reverse to align with the parking spot
            elif self.parking_status == self.PARKING_STATE_SEARCHING:
                if frontFarRightDistance < self.MIN_PARKING_SIZE_SPOT and rearFarRightDistance < self.MIN_PARKING_SIZE_SPOT and not self.autoparkstate:
                    print("Potential parking spot found")
                    self.send_data(self.CONTROL_FORWARD, 20)
                    self.autoparkstate = True

                elif frontFarRightDistance < self.MIN_PARKING_SIZE_SPOT and rightFrontDistance > 120 and rightRearDistance > 120 and self.autoparkstate:
                    print("Parking spot opening detected")
                    self.send_data(self.CONTROL_FORWARD, 20)
                    self.autoparkstate2 = True

                elif rightFrontDistance < 250 and rightRearDistance < 250 and self.autoparkstate2:
                    print("Parking spot has been found")
                    self.parking_status = self.PARKING_STATE_ALLIGN
                    self.send_data(self.CONTROL_STOP, 0)
                    self.autoparkstate2 = False
                    self.autoparkstate = False
                    self.alignment_phase = 0
                else:
                    self.send_data(self.CONTROL_FORWARD, 10)
                    print("State: SEARCHING")
                    print(f"autoparkstate: {self.autoparkstate}, autoparkstate2: {self.autoparkstate2}")
            # Third state is aligning the car with the parking spot
            # the car will move forward until it is past the parking spot
            # then it will reverse at an angle to align with the parking spot
            elif self.parking_status == self.PARKING_STATE_ALLIGN:
                if not hasattr(self, 'alignment_phase'):
                    self.alignment_phase = 0
            
                # in the inital phase of allignment, the car will move forward until it is past the parking spot
                # like the real life manuever, the car will move forward until it is past the parking spot
                if self.alignment_phase == 0:
                    if (rightFrontDistance < self.MIN_PARKING_SIZE_SPOT and
                        rightRearDistance < self.MIN_PARKING_SIZE_SPOT and
                        rearFarRightDistance > self.MIN_PARKING_SIZE_SPOT):
                        print("Initial alignment complete - preparing for reverse maneuver")
                        self.send_data(self.CONTROL_STOP, 0)
                        self.alignment_phase = 1
            
                    else:
                        self.send_data(self.CONTROL_FORWARD, 10)
                        print("State: ALLIGNING - Moving forward past spot")
                # In the second phase, the car will reverse at an angle to align with the parking spot
                # the car will revesreat this angle until the car behind is visible in the left mirror
                # in order to reverse straight, the car will use the rear far left sensor
                elif self.alignment_phase == 1:
                    right_distance_delta = rightFrontDistance - rightRearDistance
                    
                    # Safety checks
                    safety_margin = 40 
                    is_obstacle_detected = (
                        rearDistance1 < safety_margin or 
                        rearDistance2 < safety_margin or
                        any(reading < 15 for reading in sensor_readings)
                    )
                    
                    car_behind_visible = rearFarLeftDistance < 420                  
                    if is_obstacle_detected:
                        print("Safety stop - obstacle detected behind vehicle")
                        self.send_data(self.CONTROL_STOP, 0)
                        self.alignment_phase = 2
                    elif car_behind_visible :
                        print(f"Reverse-right phase complete - Car behind is visible in left mirror")
                        print(f"Rear Far left sensor reading: {rearFarLeftDistance}cm")
                        self.send_data(self.CONTROL_STOP, 0)
                        self.alignment_phase = 2
                    else:
                        self.send_data(self.CONTROL_REVERSE_RIGHT, 40)
                        print(f"STATE: REVERSE RIGHT - Backing at angle until car behind is visible")
                        print(f"Rear Far left sensor reading: {rearFarLeftDistance}cm (waiting for <420cm)")
                # In the third phase, the car will reverse straight until the right position is reached
                # the car will reverse straight until the rear far left sensor reading is less than 300cm
                # thenthe car will beging reversing left to align with the curb
                elif self.alignment_phase == 2:
                    safety_margin = 30  
                    is_obstacle_detected = (
                        rearDistance1 < safety_margin or 
                        rearDistance2 < safety_margin 
                    )
                    right_position_reached = rearFarLeftDistance < 260  
                    if is_obstacle_detected:
                        print("Safety stop - obstacle detected behind vehicle")
                        self.send_data(self.CONTROL_STOP, 0)
                        self.alignment_phase = 3

                    elif right_position_reached :
                        print(f"Straight reversing complete - Right position reached")
                        print(f"Rear Far Left distance: {rearFarLeftDistance}cm")
                        self.send_data(self.CONTROL_STOP, 0)
                        self.alignment_phase = 3

                    else:
                        self.send_data(self.CONTROL_BACKWARD, 30)  
                        print(f"STATE: REVERSE STRAIGHT - Until right position")
                        print(f"Rear Far left distance: {rearFarLeftDistance}cm (waiting for <300cm)")

                # In the fourth phase, the car will reverse left to align with the curb
                # then it will will enter the final phase of alignment in order to make sure that the car is alligned
                # correctly with the curb and the distance between the cars in parking slot is acceptable
                elif self.alignment_phase == 3:
                    safety_margin = 35 
                    is_obstacle_detected = (
                        rearDistance1 < safety_margin or 
                        rearDistance2 < safety_margin 
                    )
                    right_distance_delta = abs(rightFrontDistance - rightRearDistance)
                    right_avg = (rightFrontDistance + rightRearDistance) / 2
                    is_parallel = right_distance_delta < 30 
                    is_positioned = right_avg > 150 and right_avg < 300 
                    if is_obstacle_detected:
                        print("Safety stop - obstacle detected while parking")
                        self.send_data(self.CONTROL_STOP, 0)
                        self.alignment_phase = 4
                    elif (is_parallel and is_positioned ):
                        print(f"Final position reached - Parallel with curb")
                        print(f"Right side difference: {right_distance_delta:.1f}cm, avg: {right_avg:.1f}cm")
                        self.send_data(self.CONTROL_STOP, 0)
                        self.alignment_phase = 4 

                    else:
                        self.send_data(self.CONTROL_REVERSE_LEFT, 35)
                        print(f"STATE: REVERSE LEFT - Final positioning")
                        print(f"Right side difference: {right_distance_delta:.1f}cm, avg: {right_avg:.1f}cm")

                # In the final phase, the car will use the front and rear center sensors to align with the curb
                # the car will check if the front and rear sensors are aligned correctly to make sure
                # that there isn't any part of car outside the parking slot
                # this maneuver and algorthim esnures that there is no need to have wall or curb on the right side
                # to park the car correctly that way the car can park in any parking slot
                # this makes our algorithm more robust and flexible and of course more realistic, genuine and practical
                # although it might cost more ultrasonic sensors but it is worth it
                elif self.alignment_phase == 4:
                    front_left = frontDistance1  
                    front_right = frontDistance2  
                    rear_left = rearDistance1 
                    rear_right = rearDistance2 
                    front_diff = abs(front_left - front_right)
                    rear_diff = abs(rear_left - rear_right)

                    min_safe = 25
                    rear_too_close = min(rear_left, rear_right) < min_safe and min(rear_left, rear_right) > 0
                    front_too_close = min(front_left, front_right) < min_safe and min(front_left, front_right) > 0

                    print(f"Front sensors: {front_left}cm, {front_right}cm (diff: {front_diff}cm)")
                    print(f"Rear sensors: {rear_left}cm, {rear_right}cm (diff: {rear_diff}cm)")
                    # in this state we check if the car is too close to the car behind it
                    if rear_too_close:
                        print(f"SAFETY: Moving forward - rear distance critical")
                        self.send_data(self.CONTROL_FORWARD, 15)
                    # if the car is too close to the front vehicle, we will move backward
                    elif front_too_close:
                        print(f"SAFETY: Moving backward - front distance critical")
                        self.send_data(self.CONTROL_BACKWARD, 15)
                    
                    # now checking if the car is misaligned with the parking slot
                    elif front_diff > 3 :
                        if front_left > front_right:
                            print(f"ALIGNMENT: Forward-right to align with front vehicle")
                            self.send_data(self.CONTROL_FORWARD_RIGHT, 10)
                        else:
                            print(f"ALIGNMENT: Forward-left to align with front vehicle")
                            self.send_data(self.CONTROL_FORWARD_LEFT, 10)       
                    elif rear_diff > 4 :
                        if rear_left > rear_right:
                            print(f"ALIGNMENT: Forward-left to align with rear vehicle")
                            self.send_data(self.CONTROL_REVERSE_RIGHT, 10)
                        else:
                            print(f"ALIGNMENT: Forward-right to align with rear vehicle")
                            self.send_data(self.CONTROL_REVERSE_LEFT, 10)
                            
                    else:
                        print(f"Parking complete! Position is acceptable.")
                        print(f"Final front sensors: {front_left}cm, {front_right}cm")
                        print(f"Final rear sensors: {rear_left}cm, {rear_right}cm")
                        self.parking_status = self.PARKING_STATE_FINISHED
                        self.send_data(self.CONTROL_STOP, 0)

            # Final state is when the parking is finished
            # the car will stop and notify the user that the parking is finished
            elif self.parking_status == self.PARKING_STATE_FINISHED:
                self.parking_status = self.PARKING_STATE_IDLE
                print("Auto-parking completed!")
                self.send_data(self.CONTROL_STOP, 0)
                self.parking_active = False
                self.hud.notification("Auto-parking completed!")
                if hasattr(self, 'alignment_phase'):
                    self.alignment_phase = 0

            else:
                print("Error: Unexpected state!")
                self.parking_status = self.PARKING_STATE_IDLE
                self.send_data(self.CONTROL_STOP, 0) 


    def process_perpendicular_parking(self, sensor_readings, vehicle_speed):
        '''
        * Service Name: process_perpendicular_parking
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): sensor_readings, vehicle_speed
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: Processes sensor readings and vehicle speed for perpendicular parking.
        '''

        frontDistance1 = sensor_readings[0]
        frontDistance2 = sensor_readings[1]
        frontFarLeftDistance = sensor_readings[2]
        frontFarRightDistance = sensor_readings[3]
        rearDistance1 = sensor_readings[4]
        rearDistance2 = sensor_readings[5]
        rearFarLeftDistance = sensor_readings[6]
        rearFarRightDistance = sensor_readings[7]
        leftFrontDistance = sensor_readings[8]
        leftRearDistance = sensor_readings[9]
        rightFrontDistance = sensor_readings[10]
        rightRearDistance = sensor_readings[11]
        
        if self.debug_mode:
            print(f"PERPENDICULAR PARKING - Current state: {self.parking_status}")
            print(f"Front: {frontDistance1}, {frontDistance2}, Front Far L/R: {frontFarLeftDistance}, {frontFarRightDistance}")
            print(f"Rear: {rearDistance1}, {rearDistance2}, Rear Far L/R: {rearFarLeftDistance}, {rearFarRightDistance}")
            print(f"Side: Left F/R: {leftFrontDistance}, {leftRearDistance}, Right F/R: {rightFrontDistance}, {rightRearDistance}")
        
        # as the initail state is always idle
        # in this state the car will wait for the user to activate the perpendicular parking
        # the car will enter prep searching state to find a parking spot
        # it will be divided into 5 phases
        if self.parking_status == self.PARKING_STATE_PERP_SEARCHING:
            if not hasattr(self, 'perp_phase'):
                self.perp_phase = 0
                self.spot_detected = False
                self.spot_entry_detected = False
                self.spot_passing = False
                print("PERP PARKING: Initialized perpendicular parking detection variables")
            
            # in First phase: Looking for a potential spot by checking the angled front right sensor
            if not self.spot_detected and frontFarRightDistance > 250 and rightFrontDistance >300:
                self.spot_detected = True
                print(f"PERP PARKING: Potential spot entry detected (frontFarRight: {frontFarRightDistance}cm)")
                self.send_data(self.CONTROL_FORWARD, 15)
            
            # in Second phase: in this phase we scan the parking spot depth to check if it is big enough
            elif self.spot_detected and not self.spot_entry_detected and rightFrontDistance > 200:
                self.spot_entry_detected = True
                print(f"PERP PARKING: Spot entry confirmed (rightFront: {rightFrontDistance}cm)")
                self.send_data(self.CONTROL_FORWARD, 15)
            
            # in Third phase: we make sure that we have passed the spot in order to reverse into it
            # and start Reverse L maneuver
            elif self.spot_entry_detected and not self.spot_passing and rightRearDistance > 200 :
                self.spot_passing = True
                print(f"PERP PARKING: Now passing the spot (rightRear: {rightRearDistance}cm)")
                self.send_data(self.CONTROL_FORWARD, 15)
            
            #in Fourth phase: Confirm we've passed the spot completely.
            elif self.spot_passing and rightRearDistance < 100 and rearFarRightDistance >300:
                print(f"PERP PARKING: Completely passed the spot - preparing to position")
                print(f"  Readings: rightFront={rightFrontDistance}cm, rightRear={rightRearDistance}cm")
                self.parking_status = self.PARKING_STATE_PERP_POSITIONING
                self.perp_phase = 0
                self.send_data(self.CONTROL_STOP, 0)
            
            # in Fifth phase: if we didn't find a spot, we will keep moving forward
            else:
                self.send_data(self.CONTROL_FORWARD, 15)
                if self.spot_detected and frontFarRightDistance < 200 and rightFrontDistance < 150 and not self.spot_entry_detected:
                    print("PERP PARKING: Lost track of potential spot - resetting detection")
                    self.spot_detected = False
                    self.spot_entry_detected = False
                    self.spot_passing = False
                print("Searching for perpendicular parking spot...")
        
        # in the state of perpendicular parking positioning 
        # in which we make sure that the car is positioned correctly to start the maneuver
        # the car will move forward until the right rear sensor is close to the curb or wall
        # and the right front sensor is far enough to allow the car to turn
        elif self.parking_status == self.PARKING_STATE_PERP_POSITIONING:
            if not hasattr(self, 'perp_phase'):
                self.perp_phase = 0 
            if self.perp_phase == 0:
                if rightRearDistance < 150 and rightFrontDistance > 200:
                    print("Positioned for backward entry - preparing to turn")
                    self.perp_phase = 1
                    self.send_data(self.CONTROL_STOP, 0)
                else:
                    self.send_data(self.CONTROL_FORWARD, 10)   
            elif self.perp_phase == 1:
                self.parking_status = self.PARKING_STATE_PERP_MANEUVERING
                self.perp_phase = 0
                print("Starting backward perpendicular parking maneuver")
        # in the state of perpendicular parking maneuvering
        # in which we will make the car enter the parking spot by reversing at an angle
        # the car will use the rear right sensor to check if the rear part of the car is in the parking spot   
        elif self.parking_status == self.PARKING_STATE_PERP_MANEUVERING:
            if not hasattr(self, 'perp_phase'):
                self.perp_phase = 0
            # in the initial phase of perpendicular parking maneuvering
            # the car will turn right and reverse until the rear part of the car is in the parking spot
            if self.perp_phase == 0:
                self.send_data(self.CONTROL_STEER_RIGHT, 100)
                self.perp_phase = 1
                print("Setting steering for backward entry")
            # in the second phase of perpendicular parking maneuvering
            # the car will reverse right until the rear part of the car is in the parking spot
            elif self.perp_phase == 1:
                if rearDistance1 < 30 or rearDistance2 < 30:
                    self.send_data(self.CONTROL_STOP, 0)
                    self.parking_status = self.PARKING_STATE_PERP_ADJUSTING
                    self.perp_phase = 0
                    print("Rear obstacle detected - stopping for safety")
                else:
                    self.send_data(self.CONTROL_REVERSE_RIGHT, 40)
                    # Checking if the rear part of the car is in the parking spot
                    if leftRearDistance<140 and rightRearDistance < 120:
                        self.send_data(self.CONTROL_STOP, 0)
                        self.perp_phase = 2
                        print("Backing into spot - preparing for final adjustments")
            # in the third phase of perpendicular parking maneuvering
            # the car will reverse straight until the rear part of the car is in the parking spot
            # the car will use the rear right sensor to check if the rear part of the car is in the parking spot
            elif self.perp_phase == 2:
                self.send_data(self.CONTROL_BACKWARD, 20)
                if abs(rightFrontDistance - rightRearDistance) < 20:
                    self.send_data(self.CONTROL_STOP, 0)
                    self.parking_status = self.PARKING_STATE_PERP_ADJUSTING
                    self.perp_phase = 0
                    print("Vehicle straightened in spot - preparing final adjustments")
                    
        # in the state of perpendicular parking adjusting
        # this state is responsible for making sure that the car is positioned correctly in the parking spot
        elif self.parking_status == self.PARKING_STATE_PERP_ADJUSTING:
            if not hasattr(self, 'perp_phase'):
                self.perp_phase = 0
               
            if not hasattr(self, 'adjustment_attempts'):
                self.adjustment_attempts = 0
                
            # here we try to check the difference between the front and rear distance
            # to make sure if it is parallel to the other cars and in the parking spot
            right_diff = abs(rightFrontDistance - rightRearDistance)
            front_clearance = min(frontDistance1, frontDistance2)
            rear_clearance = min(rearDistance1, rearDistance2)
        
            print(f"Adjustment status: front_right={rightFrontDistance}cm, rear_right={rightRearDistance}cm, diff={right_diff}cm")
            print(f"Clearances: front={front_clearance}cm, rear={rear_clearance}cm")
            
            # we check here if the car is well positioned in the parking spot without being
            # to close to the wall or other car
            if (right_diff < 20 and 
                rightFrontDistance > 30 and rightFrontDistance < 80 and
                rightRearDistance > 30 and rightRearDistance < 80 and
                rear_clearance > 30):
                # Well positioned in the spot
                print("Vehicle successfully parked in perpendicular spot!")
                self.send_data(self.CONTROL_STOP, 0)
                self.parking_status = self.PARKING_STATE_FINISHED
            else:
                # Now we will have some adjustment limits to make sure that the car is 
                # in the parking spot ensuring that the driver is capable of parking and 
                # opening doors of the car without problem
                self.adjustment_attempts += 1
                if self.adjustment_attempts > 25: 
                    print("Maximum adjustment attempts reached - finalizing parking")
                    self.send_data(self.CONTROL_STOP, 0)
                    self.parking_status = self.PARKING_STATE_FINISHED
                    return
                    
                # if the car is inclined in which the right front is ver close to the other car or wall
                # while the rear side of the car is not close to the wall or other car
                #  this means that the car is entered with inlcined angle making it angled parking
                if rightFrontDistance > 120 and rightRearDistance < 140 and right_diff > 30:
                    if not hasattr(self, 'adjustment_sequence') or self.adjustment_sequence == 0:
                        print("Front of car too far out - executing special adjustment: reverse-left")
                        self.send_data(self.CONTROL_REVERSE_LEFT, 20)
                        self.adjustment_sequence = 1
                    # here we will check if the rear side of the car is in the parking spot and the front side is not
                    elif self.adjustment_sequence == 1 and rear_clearance < 50:
                        print("Now pulling front of car into spot with forward-right")
                        self.send_data(self.CONTROL_FORWARD_RIGHT, 25) 
                        self.adjustment_sequence = 2
                    # the car is now in the parking spot and we need to make sure that the front side is not too close to the wall or other car
                    elif self.adjustment_sequence == 2:
                        if rightFrontDistance < 100:
                            print("Front position improved - returning to standard adjustments")
                            self.adjustment_sequence = 0
                        else:
        
                            self.send_data(self.CONTROL_FORWARD_RIGHT, 25)
                    else:
                        self.send_data(self.CONTROL_FORWARD_RIGHT, 25)
                        
                # the car now is in the parking spot but the rear side is too close to the wall or other car
                elif rear_clearance < 30:
                    self.send_data(self.CONTROL_FORWARD, 15)
                    print("Adjusting: Too close to rear wall")
                elif right_diff > 15:
                    # the car isn't on the parallel postion
                    if rightFrontDistance > rightRearDistance:
                        self.send_data(self.CONTROL_FORWARD_LEFT, 15)
                        print("Adjusting angle: front-left")
                    else:
                        self.send_data(self.CONTROL_REVERSE_LEFT, 15)
                        print("Adjusting angle: reverse-left")
                # the car is in the parking spot but the front and rear sides is too close to the wall or other car
                elif rightFrontDistance < 30 or rightRearDistance < 30:
                    self.send_data(self.CONTROL_FORWARD_LEFT, 15)
                    print("Adjusting: Too close to right wall")
                # the car is in the parking spot but the front and rear sides is too far from the wall or other car
                elif rightFrontDistance > 80 or rightRearDistance > 80:
                    self.send_data(self.CONTROL_FORWARD_RIGHT, 25)
                    print("Adjusting: Too far from right wall (enhanced correction)")
                else:
                    # the car is now in acceptable position and it will be stopped
                    self.send_data(self.CONTROL_STOP, 0)
                    self.parking_status = self.PARKING_STATE_FINISHED
                    print("Parking complete - position acceptable")
        elif self.parking_status == self.PARKING_STATE_FINISHED:
            # prep parking will be completed insha allah and finally the car will be stopped
            print("Backward perpendicular auto-parking completed!")
            self.send_data(self.CONTROL_STOP, 0)
            self.parking_active = False
            self.hud.notification("Backward perpendicular auto-parking completed!")

    '''
    * Service Name: send_data
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): control, value
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: Sends control commands to the vehicle based on the specified control type and value.
    * there is problem in carla when running it on client side
    the phyics is not working properly
    * so we need to stop the car first and then apply the control
    '''
    def send_data(self, control, value):
     
        vehicle = self.parent_actor
        current_control = vehicle.get_control()
        current_control.manual_gear_shift = True
        
        if control == self.CONTROL_FORWARD:
            stopping_control = carla.VehicleControl()
            stopping_control.throttle = 0.0
            stopping_control.brake = 1.0
            stopping_control.manual_gear_shift = True
            stopping_control.gear = 0  
            vehicle.apply_control(stopping_control)
            time.sleep(0.1)  
            current_control.throttle = max(0.3, value / 100.0)
            current_control.brake = 0.0
            current_control.steer = 0.0
            current_control.reverse = False
            current_control.gear = 1  
            self.hud.notification(f"Moving forward at {value}% throttle")

        elif control == self.CONTROL_BACKWARD:
            stopping_control = carla.VehicleControl()
            stopping_control.throttle = 0.0
            stopping_control.brake = 1.0
            stopping_control.manual_gear_shift = True
            stopping_control.gear = 0  
            vehicle.apply_control(stopping_control)
            time.sleep(0.1)  
            current_control.throttle = min(1.0, value / 100.0)
            current_control.brake = 0.0
            current_control.reverse = True
            current_control.manual_gear_shift = True
            current_control.gear = -1 
            current_control.steer = 0.0  
            self.hud.notification(f"Reversing straight at {value}% throttle")
            
        elif control == self.CONTROL_STOP:
            current_control.throttle = 0.0
            current_control.brake = 1.0
            current_control.steer = 0.0
            current_control.gear = 0  
            self.hud.notification("Stopping vehicle")
            
        elif control == self.CONTROL_STEER_LEFT:
            stopping_control = carla.VehicleControl()
            stopping_control.throttle = 0.0
            stopping_control.brake = 1.0
            stopping_control.manual_gear_shift = True
            stopping_control.gear = 0 
            vehicle.apply_control(stopping_control)
            time.sleep(0.1) 
            current_control.throttle = 0.2
            current_control.brake = 0.0
            current_control.steer = -0.5
            current_control.reverse = False
            current_control.gear = 1 
            self.hud.notification("Steering left")
            
        elif control == self.CONTROL_STEER_RIGHT:
            stopping_control = carla.VehicleControl()
            stopping_control.throttle = 0.0
            stopping_control.brake = 1.0
            stopping_control.manual_gear_shift = True
            stopping_control.gear = 0 
            vehicle.apply_control(stopping_control)
            time.sleep(0.1)
            current_control.throttle = 0.2
            current_control.brake = 0.0
            current_control.steer = 0.5
            current_control.reverse = False
            current_control.gear = 1
            self.hud.notification("Steering right")
            
        elif control == self.CONTROL_FORWARD_LEFT:
            stopping_control = carla.VehicleControl()
            stopping_control.throttle = 0.0
            stopping_control.brake = 1.0
            stopping_control.manual_gear_shift = True
            stopping_control.gear = 0
            vehicle.apply_control(stopping_control)
            time.sleep(0.1) 
            current_control.throttle = min(1.0, value / 100.0)
            current_control.brake = 0.0
            current_control.reverse = False
            current_control.steer = -0.8
            current_control.gear = 1 
            self.hud.notification(f"Moving forward-left at {value}% throttle")
        
        elif control == self.CONTROL_FORWARD_RIGHT:
            stopping_control = carla.VehicleControl()
            stopping_control.throttle = 0.0
            stopping_control.brake = 1.0
            stopping_control.manual_gear_shift = True
            stopping_control.gear = 0
            vehicle.apply_control(stopping_control)
            time.sleep(0.1)
            current_control.throttle = min(1.0, value / 100.0)
            current_control.brake = 0.0
            current_control.reverse = False
            current_control.steer = 0.8
            current_control.gear = 1 
            self.hud.notification(f"Moving forward-right at {value}% throttle")
        
        elif control == self.CONTROL_REVERSE_LEFT:

            stopping_control = carla.VehicleControl()
            stopping_control.throttle = 0.0
            stopping_control.brake = 1.0
            stopping_control.manual_gear_shift = True
            stopping_control.gear = 0
            vehicle.apply_control(stopping_control)
            time.sleep(0.1)
            current_control.manual_gear_shift = True
            current_control.gear = -1  
            current_control.throttle = 0.7
            current_control.brake = 0.0
            current_control.reverse = True
            current_control.steer = -0.7
            self.hud.notification("Reversing left")
            
        elif control == self.CONTROL_REVERSE_RIGHT:
   
            stopping_control = carla.VehicleControl()
            stopping_control.throttle = 0.0
            stopping_control.brake = 1.0
            stopping_control.manual_gear_shift = True
            stopping_control.gear = 0  
            vehicle.apply_control(stopping_control)
            time.sleep(0.1)
            current_control.manual_gear_shift = True
            current_control.gear = -1 
            current_control.throttle = 0.6
            current_control.brake = 0.0
            current_control.reverse = True
            current_control.steer = 0.8
            self.hud.notification("Reversing right")

        elif control == self.CONTROL_FORWARD_RIGHT_PREP:
            stopping_control = carla.VehicleControl()
            stopping_control.throttle = 0.0
            stopping_control.brake = 1.0
            stopping_control.manual_gear_shift = True
            stopping_control.gear = 0 
            vehicle.apply_control(stopping_control)
            time.sleep(0.1)
            current_control.throttle = min(1.0, value / 100.0)
            current_control.brake = 0.0
            current_control.reverse = False
            current_control.steer = 0.7
            current_control.gear = 1  
            self.hud.notification(f"Moving forward-right at {value}% throttle")

        elif control == self.CONTROL_FORWARD_LEFT_PREP:
            stopping_control = carla.VehicleControl()
            stopping_control.throttle = 0.0
            stopping_control.brake = 1.0
            stopping_control.manual_gear_shift = True
            stopping_control.gear = 0  
            vehicle.apply_control(stopping_control)
            time.sleep(0.1)
            current_control.throttle = min(1.0, value / 100.0)
            current_control.brake = 0.0
            current_control.reverse = False
            current_control.steer = -0.7
            current_control.gear = 1  
            self.hud.notification(f"Moving forward-left at {value}% throttle")

        vehicle.apply_control(current_control)

        try:
            self.parent_actor.get_world().tick()
        except Exception as e:
            print(f"Error during tick: {e}")
    
        print(f"Applied control: throttle={current_control.throttle}, brake={current_control.brake}, "
            f"steer={current_control.steer}, reverse={current_control.reverse}, "
            f"manual_gear_shift={current_control.manual_gear_shift}, gear={current_control.gear}")

