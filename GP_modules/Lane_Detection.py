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
def _process_lane_detection(self, image):
    '''
    * Service Name: _process_lane_detection
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): image
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to process the lane detection
    '''
    if hasattr(self._parent, 'get_world') and hasattr(self._parent.get_world(), 'ultrasonic_sensors') and \
        hasattr(self._parent.get_world().ultrasonic_sensors, 'auto_parking_active') and \
        self._parent.get_world().ultrasonic_sensors.auto_parking_active:
            height, width = image.shape[:2]
            frame = image.copy()
            cv2.putText(frame, "AUTO PARKING ACTIVE", (width//2-150, height//2), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.lane_surface = pygame.surfarray.make_surface(frame.swapaxes(0, 1))
            return
    frame = image.copy()
    height, width = frame.shape[:2]
    

    def lanes_detection(img):
        '''
        * Service Name: lanes_detection
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): img
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: left_line, right_line, (height, width)
        * Description: to detect lanes in the image and return the left and right lane lines
        '''
        grayscale_image = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        blurred_image = cv2.GaussianBlur(grayscale_image, (5, 5), 0)
        edge_image = cv2.Canny(blurred_image, threshold1=50, threshold2=150, apertureSize=3)
        roi_corners = [(0, height), (width // 2, height // 2), (width, height)]
        roi_mask = np.zeros_like(edge_image)
        height, width = img.shape[:2]
        cv2.fillPoly(roi_mask, np.array([roi_corners], dtype=np.int32), 255)
        masked_edges = cv2.bitwise_and(edge_image, roi_mask)
        lines = cv2.HoughLinesP(masked_edges, rho=1, theta=np.pi/180, threshold=50,minLineLength=15, maxLineGap=300)
        left_lines, right_lines = [], []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1 + 1e-9)
                if slope < -0.2:
                    left_lines.append(line)
                elif slope > 0.2:
                    right_lines.append(line)

        def average_line(lines):
            '''
            * Service Name: average_line
            * Sync/Async: Synchronous
            * Reentrancy: Non-Reentrant
            * Parameters (in): lines
            * Parameters (inout): None
            * Parameters (out): None
            * Return value: average line
            * Description: to calculate the average line from the detected lines
            *              and return the average line coordinates
            '''
            if not lines:
                return [0, 0, 0, 0]
            start_x = sum(line[0][0] for line in lines) / len(lines)
            start_y = sum(line[0][1] for line in lines) / len(lines)
            end_x = sum(line[0][2] for line in lines) / len(lines)
            end_y = sum(line[0][3] for line in lines) / len(lines)
            return [int(start_x), int(start_y), int(end_x), int(end_y)]
            
        left_line = average_line(left_lines)
        right_line = average_line(right_lines)
        return left_line, right_line, (height, width)

    left_line, right_line, (height, width) = lanes_detection(frame)
    if left_line == [0, 0, 0, 0]:
        if self.left_lane_history:
            left_line = self.left_lane_history[-1]
    else:
        self.left_lane_history.append(left_line)
        if len(self.left_lane_history) > self.history_length:
            self.left_lane_history.pop(0)
    
    if right_line == [0, 0, 0, 0]:
        if self.right_lane_history:
            right_line = self.right_lane_history[-1]
    else:
        self.right_lane_history.append(right_line)
        if len(self.right_lane_history) > self.history_length:
            self.right_lane_history.pop(0)
    

    def extend_line(line, img_height):
        '''
        * Service Name: extend_line
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): line, img_height
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: extended line
        * Description: to extend the line to the bottom of the image
        '''
        if line == [0, 0, 0, 0]:
            return line
        x1, y1, x2, y2 = line
        slope = (y2 - y1) / (x2 - x1 + 1e-6)
        intercept = y1 - slope * x1
        y_bottom = img_height
        x_bottom = int((y_bottom - intercept) / slope)
        y_top = int(img_height * 0.6)
        x_top = int((y_top - intercept) / slope)
        return [x_bottom, y_bottom, x_top, y_top]
    left_line = extend_line(left_line, height)
    right_line = extend_line(right_line, height)
    thickness = 9
    if left_line != [0, 0, 0, 0]:
        cv2.line(frame, (left_line[0], left_line[1]), (left_line[2], left_line[3]), (255, 0, 0), thickness)
    if right_line != [0, 0, 0, 0]:
        cv2.line(frame, (right_line[0], right_line[1]), (right_line[2], right_line[3]), (255, 0, 0), thickness)
    lane_data = None
    car_position = width // 2
    if left_line != [0, 0, 0, 0] and right_line != [0, 0, 0, 0]:
        lane_center = (left_line[0] + right_line[0]) // 2
        deviation = car_position - lane_center
        cv2.line(frame, (lane_center, height), (lane_center, height-50), (0, 255, 0), 2)
        
        # this is our package lane data for the warning thread
        lane_data = {
            'deviation': deviation,
            'width': width,
            'left_line': left_line,
            'right_line': right_line,
            'timestamp': time.time()
        }
        
        with self.lane_state_lock:
            if self.crossing_state['right']:
                cv2.putText(frame, "RIGHT CROSS", (width-150, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            elif self.crossing_state['left']:
                cv2.putText(frame, "LEFT CROSS", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                cv2.putText(frame, "CENTER", (width//2-40, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            world = self._parent.get_world()
            if hasattr(world, 'lka_active') and world.lka_active:
                cv2.putText(frame, "LKA ACTIVE", (width//2-60, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    
    if hasattr(self, 'traffic_sign_detector') and self.traffic_sign_detector:
        stable_signs = self.traffic_sign_detector.stable_detections
        for class_id, (confidence, (x, y, w, h)) in stable_signs.items():
            sign_name = self.traffic_sign_detector.classes.get(class_id, "Unknown")
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, f"{sign_name}: {confidence:.2f}", 
                    (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        if stable_signs:
            for class_id, (confidence, _) in list(stable_signs.items())[:1]:  # Show first sign
                sign_name = self.traffic_sign_detector.classes.get(class_id, "Unknown")
                self.hud.notification(f"Traffic Sign: {sign_name}", seconds=1.0)

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    self.lane_surface = pygame.surfarray.make_surface(frame.swapaxes(0, 1))
    if lane_data:
        try:
            self.lane_data_queue.put(lane_data, block=True, timeout=0.01)
        except queue.Full:
            try:
                self.lane_data_queue.get_nowait()
                self.lane_data_queue.put(lane_data, block=False)
            except:
                pass 