#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    CTRL + W     : toggle constant velocity mode at 60 km/h

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    O            : open/close all doors of vehicle
    T            : toggle vehicle's telemetry

    V            : Select next map layer (Shift+V reverse)
    B            : Load current selected map layer (Shift+B to unload)

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit

    F3           : toggle auto parking
    F6           : send manual emergency call to V2V network
    K            : toggle LKA

"""

from __future__ import print_function


"""
this file is modified by OptiDrive. The team is a group of Automotive Embedded Software Engineers and Machine Learing Engineers

"""

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


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
try:
    from serial import Serial as SerialPort
except ImportError:
    print("PySerial module not found")

sys.path.append('d:/CarlaSim/PythonAPI/examples/GP_modules')
try:
    from Traffic_recognition import TrafficSignDetector
except ImportError:
    print("Error importing Traffic_recognition module. Make sure the path is correct.")


sys.path.append('d:/CarlaSim/PythonAPI/examples/GP_modules')
try:
    from DMS import DMSModule
except ImportError:
    print("Error importing DMSModule. Make sure the path is correct.")

sys.path.append('d:/CarlaSim/PythonAPI/examples/GP_modules')
try:
    from v2v_adhoc_network import V2VAdHocNetwork, V2VMessage, MessageType, MessagePriority
except ImportError:
    print("Error importing V2VAdHocNetwork module. Make sure the path is correct.")
try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_b
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_f
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_o
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_t
    from pygame.locals import K_v
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_z
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
    from pygame.locals import K_F3
    from pygame.locals import K_k
    from pygame.locals import K_F6
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')



# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================

lka_active_confirm = True
def play_buzzer():
    # Play a 500Hz sound for 100ms
    winsound.Beep(500, 100)

def async_buzzer():
    thread = threading.Thread(target=play_buzzer)
    thread.start()

def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud, args):
        self.world = carla_world
        self.sync = args.sync
        self.actor_role_name = args.rolename
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.ultrasonic_sensors = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self._gamma = args.gamma
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.constant_velocity_enabled = False
        self.show_vehicle_telemetry = False
        self.doors_are_open = False
        self.current_map_layer = 0
        '''----------------DMS------------------'''
        self.dms = None
        self.dms_blinker_active = False
        self.control_override_active = False
        '''----------------LKA------------------'''
        self.lka_enabled = True
        self.lka_active = False  
        self.lka_strength = 0.99
        self.lka_last_correction_time = 0
        '''----------------EBS------------------'''
        self.emergency_braking_system = None

        '''----------------V2V------------------'''
        self.v2v_network = V2VAdHocNetwork(self.world, max_range=300.0)
        self.v2v_messages = []
        self.last_v2v_update = 0
        self.v2v_update_interval = 0.2  
        self.v2v_visualization_active = True
        self.nearby_v2v_vehicles = []
        self.emergency_message_sent = False
        self.last_emergency_broadcast = 0
        self.emergency_cooldown = 30  

        self.map_layer_names = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]
        try:
            self.dms = DMSModule()
            self.hud.notification('Driver Monitoring System activated')
            print("DMS module initialized successfully")
        except Exception as e:
            print(f"Failed to initialize DMS: {e}")
            self.dms = None

    def restart(self):
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        blueprint = self.world.get_blueprint_library().find('vehicle.lincoln.mkz_2020')
        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('terramechanics'):
            blueprint.set_attribute('terramechanics', 'true')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.ultrasonic_sensors = UltrasonicSensorSystem(self.player, self.hud)
        self.emergency_braking_system = EmergencyBrakingSystem(self.player, self.hud, max_distance=70.0)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def next_map_layer(self, reverse=False):
        self.current_map_layer += -1 if reverse else 1
        self.current_map_layer %= len(self.map_layer_names)
        selected = self.map_layer_names[self.current_map_layer]
        self.hud.notification('LayerMap selected: %s' % selected)

    def load_map_layer(self, unload=False):
        selected = self.map_layer_names[self.current_map_layer]
        if unload:
            self.hud.notification('Unloading map layer: %s' % selected)
            self.world.unload_map_layer(selected)
        else:
            self.hud.notification('Loading map layer: %s' % selected)
            self.world.load_map_layer(selected)

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player, self.hud)
        elif self.radar_sensor.sensors:
            self.radar_sensor.destroy()
            self.radar_sensor = None

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass




    def tick(self, clock):
        self.hud.tick(self, clock)

        current_time = time.time()
        if current_time - self.last_v2v_update > self.v2v_update_interval:
            self.last_v2v_update = current_time
            if not self.player:
                return
            self.v2v_network.update()
            player_location = self.player.get_location()
            player_id = self.player.id
            self.v2v_messages = self.v2v_network.recieve_messages_for_vehicle(player_id, player_location)
            for msg in self.v2v_messages:
                self._process_v2v_message(msg)
            if self.dms is not None and self.dms.is_sleeping():
                self._send_driver_emergency_v2v()
            self._find_nearby_v2v_vehicles()
            if self.v2v_visualization_active:
                self.v2v_network.update_visualizations()
                if random.random() < 0.05:
                    self.v2v_network.visualize_v2v_range(self.player)
            if self.dms is not None:
                try:
                    self.dms.update()
                except Exception as e:
                    print(f"Error in DMS update: {e}")
        if self.dms is not None:
            try:
                self.dms.update()
                if self.dms.is_sleeping():
                    if not self.dms_blinker_active:
                        # Activate both blinkers to show that the car is in emergency stop
                        current_lights = self.player.get_light_state()
                        hazard_lights = carla.VehicleLightState(carla.VehicleLightState.LeftBlinker | 
                                                carla.VehicleLightState.RightBlinker)
                        self.player.set_light_state(hazard_lights)
                        self.dms_blinker_active = True
                        self.control_override_active = True
                        self.hud.notification("DRIVER SLEEPING DETECTED - EMERGENCY STOP ACTIVATED", seconds=10.0)
                        if not hasattr(self, 'emergency_slowdown_active'):
                            self.emergency_slowdown_active = True
                            self.emergency_start_speed = self.player.get_velocity()
                            self.emergency_start_time = time.time()
                            print("Emergency slowdown activated - driver sleeping")
                        async_buzzer()
                    else:
                        if hasattr(self, 'emergency_slowdown_active') and self.emergency_slowdown_active:
                            # Calculate how much to slow down - aim to stop in 5 seconds
                            elapsed = time.time() - self.emergency_start_time
                            slow_factor = max(0, 1.0 - (elapsed / 5.0))
                            control = carla.VehicleControl()
                            control.throttle = 0.0
                            control.brake = min(1.0, 0.2 + (1.0 - slow_factor))
                            control.steer = 0.0 
                            self.player.apply_control(control)
                            if int(elapsed) != int(elapsed - 0.1):
                                async_buzzer()
                            current_speed = self.player.get_velocity()
                            speed_kmh = 3.6 * math.sqrt(current_speed.x**2 + current_speed.y**2 + current_speed.z**2)
                            
                            if speed_kmh < 0.5:
                                control.brake = 1.0
                                control.hand_brake = True
                                self.player.apply_control(control)
                else:
                    if self.dms_blinker_active:
                        # Driver is awake now, restore normal lighting
                        no_lights = carla.VehicleLightState(carla.VehicleLightState.NONE)
                        self.player.set_light_state(no_lights)
                        self.dms_blinker_active = False
                        self.control_override_active = False
                        if hasattr(self, 'emergency_slowdown_active'):
                            self.emergency_slowdown_active = False
                            self.hud.notification("Driver is awake- resuming normal operation")
            except Exception as e:
                print(f"Error in DMS update: {e}")

        if (self.ultrasonic_sensors and 
            hasattr(self.ultrasonic_sensors, 'auto_parking_active') and 
            self.ultrasonic_sensors.auto_parking_active):

            current_time = time.time()
            if (hasattr(self.ultrasonic_sensors, 'last_command_time') and 
                current_time - self.ultrasonic_sensors.last_command_time < 0.5):
                pass
        if self.sync:
            world = self.world
            snapshot = self.world.get_snapshot()
            if snapshot.frame % 10 == 0:
                if len(world.get_actors().filter(self.player.type_id)) < 1:
                    print("Warning: Actor %s not found" % self.player.type_id)
                    world.tick()
                    return
                else:
                    self.player = world.get_actors().filter(self.player.type_id)[0]
            world.tick() 

    '''
    * Service Name: _process_v2v_message
    * Sync/Async: Async
    * Reentrancy: Reentrant
    * Parameters (in): message
    * Parameters (inout): None
    * Parameters (out): None   
    * Return value: None
    * Description: to process the V2V messages received from the V2V network
    '''
    def _process_v2v_message(self, message):
        if message.priority in [MessagePriority.EMERGENCY, MessagePriority.HIGH]:
            if message.type == MessageType.DRIVER_EMERGENCY:
                reason = message.content.get('reason', 'Unknown')
                distance = self._calculate_distance_to(message.location)
                self.hud.notification(f"V2V ALERT: Driver emergency {distance:.0f}m ahead!", seconds=3.0)
                async_buzzer()
            
            elif message.type == MessageType.COLLISION:
                distance = self._calculate_distance_to(message.location)
                self.hud.notification(f"V2V ALERT: Collision {distance:.0f}m ahead!", seconds=3.0)
                async_buzzer()

    '''
    * Service Name: _calculate_distance_to
    * Sync/Async: Async
    * Reentrancy: Reentrant
    * Parameters (in): location
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: distance
    * Description: to calculate the distance between the player and the location
    '''
    def _calculate_distance_to(self, location):
        if not self.player or not location:
            return 9999.0
            
        player_loc = self.player.get_location()
        distance = math.sqrt(
            (player_loc.x - location.x)**2 + 
            (player_loc.y - location.y)**2
        )
        return distance
    '''
    * Service Name: _send_driver_emergency_v2v
    * Sync/Async: Async
    * Reentrancy: Reentrant
    * Parameters (in): None
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to send the driver emergency message to the V2V network
    '''
    def _send_driver_emergency_v2v(self):
        current_time = time.time()
        if (not self.emergency_message_sent or 
            (current_time - self.last_emergency_broadcast > self.emergency_cooldown)):
            details = {
                "emergency_type": "DRIVER_UNCONSCIOUS",
                "vehicle_id": self.player.id,
                "timestamp": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "status": "AUTOMATIC_EMERGENCY_STOP_ACTIVATED"
            }
            emergency_msg = self.v2v_network.create_emergency_message(
                self.player,
                "DRIVER UNCONSCIOUS",
                priority=MessagePriority.EMERGENCY,
                details=details
            )
            
            if self.v2v_network.broadcast_message(emergency_msg):
                self.hud.notification("V2V EMERGENCY BROADCAST SENT", seconds=3.0)
                async_buzzer()
                if self.v2v_visualization_active:
                    self.v2v_network.visualize_message_broadcast(emergency_msg)
                self.emergency_message_sent = True
                self.last_emergency_broadcast = current_time
    '''
    * Service Name: _find_nearby_v2v_vehicles
    * Sync/Async: Async
    * Reentrancy: Reentrant
    * Parameters (in): None
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to find the nearby vehicles in the V2V network
    '''
    def _find_nearby_v2v_vehicles(self):
        if not self.player:
            return
            
        vehicles = self.world.get_actors().filter('vehicle.*')
        player_location = self.player.get_location()
        v2v_range = self.v2v_network.max_range
        
        self.nearby_v2v_vehicles = []
        
        for vehicle in vehicles:
            if vehicle.id == self.player.id:
                continue
            vehicle_location = vehicle.get_location()
            dist = math.sqrt(
                (vehicle_location.x - player_location.x)**2 + 
                (vehicle_location.y - player_location.y)**2
            )
            if dist <= v2v_range:
                self.nearby_v2v_vehicles.append((vehicle, dist))

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)
        if hasattr(self, 'v2v_network') and self.player and self.v2v_visualization_active:
            player_loc = self.player.get_location()
            for vehicle, dist in self.nearby_v2v_vehicles:
                if random.random() < 0.1:
                    end_loc = vehicle.get_location()

                    start_v = carla.Location(player_loc.x, player_loc.y, player_loc.z + 1.0)
                    end_v = carla.Location(end_loc.x, end_loc.y, end_loc.z + 1.0)
                    alpha = int(255 * (1.0 - dist/self.v2v_network.max_range))
                    self.world.debug.draw_line(
                        start_v,
                        end_v,
                        0.1,  
                        carla.Color(0, 200, 200, alpha),
                        0.2 
                    )

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        if self.radar_sensor is not None:
            self.toggle_radar()
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.emergency_braking_system is not None:
            self.emergency_braking_system.destroy()
        if self.ultrasonic_sensors is not None:
            self.ultrasonic_sensors.destroy()
        if self.player is not None:
            self.player.destroy()
        if self.dms is not None:
            try:
                self.dms.destroy()
                print("DMS module cleaned up")
            except Exception as e:
                print(f"Error in DMS destroy: {e}")


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        self._ackermann_enabled = False
        self._ackermann_reverse = 1
        self._F3_pressed = False
        
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()  # Initialize _control first
            self._control.manual_gear_shift = False  # Then set its attributes
            self._control.gear = 1
            self._ackermann_control = carla.VehicleAckermannControl()
            self._lights = carla.VehicleLightState.NONE
            world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, client, world, clock, sync_mode):
        global lka_active_confirm
        if hasattr(world, 'control_override_active') and world.control_override_active:
            if isinstance(self._control, carla.VehicleControl):
                # Update control display values without applying them
                self._control = world.player.get_control()
            return False 
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYDOWN:
                if event.key == K_F3 and not self._F3_pressed:
                    self._F3_pressed = True
                    if hasattr(world, 'ultrasonic_sensors') and world.ultrasonic_sensors is not None:
                        world.ultrasonic_sensors.activate_auto_parking()
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    if self._autopilot_enabled:
                        world.player.set_autopilot(False)
                        world.restart()
                        world.player.set_autopilot(True)
                    else:
                        world.restart()
                elif event.key == K_F3:
                    self._F3_pressed = False
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_F6:
                    if hasattr(world, 'v2v_network'):
                        details = {
                            "emergency_type": "MANUAL_DISTRESS_CALL",
                            "vehicle_id": world.player.id,
                            "timestamp": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                            "message": "Driver initiated emergency call"
                        }
                        
                        # Creating emergency message
                        emergency_msg = world.v2v_network.create_emergency_message(
                            world.player,
                            "MANUAL EMERGENCY CALL",
                            priority=MessagePriority.HIGH,
                            details=details
                        )
                        
                        if world.v2v_network.broadcast_message(emergency_msg):
                            world.hud.notification("Manual Emergency Call Broadcast", seconds=3.0)
                            async_buzzer()
                            
                            if world.v2v_visualization_active:
                                world.v2v_network.visualize_message_broadcast(emergency_msg)
                elif event.key == K_v and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_map_layer(reverse=True)
                elif event.key == K_v:
                    world.next_map_layer()
                elif event.key == K_b and pygame.key.get_mods() & KMOD_SHIFT:
                    world.load_map_layer(unload=True)
                elif event.key == K_b:
                    world.load_map_layer()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_k: 
                    if lka_active_confirm is False:
                        lka_active_confirm = True
                    elif lka_active_confirm is True:
                        lka_active_confirm = False
                    if hasattr(world, 'lka_enabled'):
                        world.lka_enabled = not world.lka_enabled
                        if not world.lka_enabled:
                            if hasattr(world, 'lka_active'):
                                world.lka_active = False
                                #printing lka_active
                                print ("LKA active: %s" % world.lka_active)
                            if hasattr(world, 'lka_enabled'):
                                world.lka_enabled = False
                            if hasattr(world, 'lka_continuous_thread_active'):
                                world.lka_continuous_thread_active = False
                            if hasattr(world, 'camera_manager') and hasattr(world.camera_manager, 'lka_thread'):
                                world.camera_manager.lka_thread = None
                        
                            print("LKA disabled and all active corrections stopped")
                        else:
                            print("LKA enabled - will activate on lane departure")
                        world.hud.notification("Lane Keeping Assist %s" % 
                                        ("Enabled" if world.lka_enabled else "Disabled"))
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_g:
                    world.toggle_radar()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key == K_n:
                    world.camera_manager.next_sensor()
                elif event.key == K_w and (pygame.key.get_mods() & KMOD_CTRL):
                    if world.constant_velocity_enabled:
                        world.player.disable_constant_velocity()
                        world.constant_velocity_enabled = False
                        world.hud.notification("Disabled Constant Velocity Mode")
                    else:
                        world.player.enable_constant_velocity(carla.Vector3D(12.5, 0, 0))
                        world.constant_velocity_enabled = True
                        world.hud.notification("Enabled Constant Velocity Mode at 45 km/h")
                elif event.key == K_o:
                    try:
                        if world.doors_are_open:
                            world.player.close_door(carla.VehicleDoor.All)
                            world.doors_are_open = False
                            world.hud.notification("Closed Doors")
                        else:
                            world.player.open_door(carla.VehicleDoor.All)
                            world.doors_are_open = True
                            world.hud.notification("Opened Doors")
                    except Exception:
                        pass
                elif event.key == K_t:
                    if world.show_vehicle_telemetry:
                        world.player.show_debug_telemetry(False)
                        world.show_vehicle_telemetry = False
                        world.hud.notification("Disabled Vehicle Telemetry")
                    else:
                        try:
                            world.player.show_debug_telemetry(True)
                            world.show_vehicle_telemetry = True
                            world.hud.notification("Enabled Vehicle Telemetry")
                        except Exception:
                            pass
                elif event.key > K_0 and event.key <= K_9:
                    index_ctrl = 0
                    if pygame.key.get_mods() & KMOD_CTRL:
                        index_ctrl = 9
                    world.camera_manager.set_sensor(event.key - 1 - K_0 + index_ctrl)
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    world.camera_manager.toggle_recording()
                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                    if (world.recording_enabled):
                        client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        client.start_recorder("manual_recording.rec")
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    # stop recorder
                    client.stop_recorder()
                    world.recording_enabled = False
                    # work around to fix camera at start of replaying
                    current_index = world.camera_manager.index
                    world.destroy_sensors()
                    # disable autopilot
                    self._autopilot_enabled = False
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification("Replaying file 'manual_recording.rec'")
                    # replayer
                    client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                    world.camera_manager.set_sensor(current_index)
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start -= 10
                    else:
                        world.recording_start -= 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start += 10
                    else:
                        world.recording_start += 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_f:
                        # Toggle ackermann controller
                        self._ackermann_enabled = not self._ackermann_enabled
                        world.hud.show_ackermann_info(self._ackermann_enabled)
                        world.hud.notification("Ackermann Controller %s" %
                                              ("Enabled" if self._ackermann_enabled else "Disabled"))
                    elif event.key == K_q:
                        if not self._ackermann_enabled:
                            self._control.gear = 1 if self._control.reverse else -1
                            self._control.reverse = not self._control.reverse
                            world.hud.notification("Gear: %s" % (
                                "Reverse" if self._control.reverse else "Forward"))
                        else:
                            self._ackermann_reverse = -1 if self._ackermann_reverse > 0 else 1
                            self._ackermann_control.speed = self._ackermann_control.speed * self._ackermann_reverse
                            world.hud.notification("Ackermann Reverse: %s" % (
                                "Enabled" if self._ackermann_reverse < 0 else "Disabled"))
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' %
                                              ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                        if not self._autopilot_enabled and not sync_mode:
                            print("WARNING: You are currently in asynchronous mode and could "
                                  "experience some issues with the traffic simulation")
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification(
                            'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                        current_lights ^= carla.VehicleLightState.Special1
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                        current_lights ^= carla.VehicleLightState.HighBeam
                    elif event.key == K_l:
                        # Use 'L' key to switch between lights:
                        # closed -> position -> low beam -> fog
                        if not self._lights & carla.VehicleLightState.Position:
                            current_lights |= carla.VehicleLightState.Position
                        else:
                            current_lights |= carla.VehicleLightState.LowBeam
                        if self._lights & carla.VehicleLightState.LowBeam:
                            current_lights |= carla.VehicleLightState.Fog
                        if self._lights & carla.VehicleLightState.Fog:
                            current_lights &= ~(carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam | carla.VehicleLightState.Fog)
                    elif event.key == K_i:
                        current_lights ^= carla.VehicleLightState.Interior
                    elif event.key == K_z:
                        current_lights ^= carla.VehicleLightState.LeftBlinker
                    elif event.key == K_x:
                        current_lights ^= carla.VehicleLightState.RightBlinker

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._control.reverse = self._control.gear < 0
                # Set automatic control-related vehicle lights
                if self._control.brake:
                    current_lights |= carla.VehicleLightState.Brake
                else: # Remove the Brake flag
                    current_lights &= ~carla.VehicleLightState.Brake
                if self._control.reverse:
                    current_lights |= carla.VehicleLightState.Reverse
                else: # Remove the Reverse flag
                    current_lights &= ~carla.VehicleLightState.Reverse
                if current_lights != self._lights: # Change the light state only if necessary
                    self._lights = current_lights
                    world.player.set_light_state(carla.VehicleLightState(self._lights))
                # Apply control
                if not self._ackermann_enabled:
                    world.player.apply_control(self._control)
                else:
                    world.player.apply_ackermann_control(self._ackermann_control)
                    # Update control to the last one applied by the ackermann controller.
                    self._control = world.player.get_control()
                    # Update hud with the newest ackermann control
                    world.hud.update_ackermann_control(self._ackermann_control)

            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time(), world)
                world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        if keys[K_UP] or keys[K_w]:
            # print("Up/W key pressed - applying throttle")
            if not self._ackermann_enabled:
                self._control.throttle = min(self._control.throttle + 0.1, 1.00)
            else:
                self._ackermann_control.speed += round(milliseconds * 0.005, 2) * self._ackermann_reverse
        else:
            if not self._ackermann_enabled:
                self._control.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            if not self._ackermann_enabled:
                self._control.brake = min(self._control.brake + 0.2, 1)
            else:
                self._ackermann_control.speed -= min(abs(self._ackermann_control.speed), round(milliseconds * 0.005, 2)) * self._ackermann_reverse
                self._ackermann_control.speed = max(0, abs(self._ackermann_control.speed)) * self._ackermann_reverse
        else:
            if not self._ackermann_enabled:
                self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        if not self._ackermann_enabled:
            self._control.steer = round(self._steer_cache, 1)
            self._control.hand_brake = keys[K_SPACE]
        else:
            self._ackermann_control.steer = round(self._steer_cache, 1)

    def _parse_walker_keys(self, keys, milliseconds, world):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = world.player_max_speed_fast if pygame.key.get_mods() & KMOD_SHIFT else world.player_max_speed
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

        self._show_ackermann_info = False
        self._ackermann_control = carla.VehicleAckermannControl()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        compass = world.imu_sensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Compass:% 17.0f\N{DEGREE SIGN} % 2s' % (compass, heading),
            'Accelero: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.accelerometer),
            'Gyroscop: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.gyroscope),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '',
            'Lane Keeping Assist: %s' % ('Enabled' if world.lka_enabled else 'Disabled'),
            ('LKA Active:', world.lka_active)
            ]
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
            if self._show_ackermann_info:
                self._info_text += [
                    '',
                    'Ackermann Controller:',
                    '  Target speed: % 8.0f km/h' % (3.6*self._ackermann_control.speed),
                ]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles, key=lambda vehicles: vehicles[0]):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))
        if hasattr(world, 'v2v_network'):
            self._info_text.append('')
            self._info_text.append('V2V Ad-hoc Network:')
            self._info_text.append(f'  Range: {world.v2v_network.max_range:.0f}m')
            self._info_text.append(f'  Vehicles in range: {len(world.nearby_v2v_vehicles)}')
            self._info_text.append(f'  Messages received: {len(world.v2v_messages)}')
            
            # Show emergency status if active
            if hasattr(world, 'emergency_message_sent') and world.emergency_message_sent:
                if time.time() - world.last_emergency_broadcast < 60:
                    self._info_text.append('  EMERGENCY BROADCAST ACTIVE')
            

    def show_ackermann_info(self, enabled):
        self._show_ackermann_info = enabled

    def update_ackermann_control(self, ackermann_control):
        self._ackermann_control = ackermann_control

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


        '''-------------------this part is for emergency call message-------------------'''
        if intensity > 50:  
            world = self._parent.get_world()
            if hasattr(world, 'v2v_network'):
                details = {
                    "collision_intensity": intensity,
                    "collision_with": actor_type,
                    "impact_location": {
                        "x": event.other_actor.get_location().x,
                        "y": event.other_actor.get_location().y
                    }
                }
                
                collision_msg = world.v2v_network.create_emergency_message(
                    self._parent,
                    "VEHICLE COLLISION",
                    priority=MessagePriority.EMERGENCY,
                    details=details
                )
                
                if collision_msg and world.v2v_network.broadcast_message(collision_msg):
                    self.hud.notification("V2V COLLISION ALERT BROADCAST", seconds=3.0)
                    world.v2v_network.visualize_message_broadcast(collision_msg)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor
            self.hud = hud
            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid circular
            # reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


# ==============================================================================
# -- RadarSensor ---------------------------------------------------------------
# ==============================================================================
class RadarSensor(object):
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
    def __init__(self, parent_actor, hud):
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
    def _Radar_callback(self, radar_data, position):
        car_velocity = self._parent.get_velocity()
        car_speed = math.sqrt(car_velocity.x**2 + car_velocity.y**2 + car_velocity.z**2)
        
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            if abs(azi) <= 30 and detect.depth > 1.0: 
                relative_velocity = detect.velocity - car_speed
                if relative_velocity < -1.9:  
                    self.hud.notification(f'Vehicle approaching from {position}!')
                    print(f'Vehicle approaching from {position}: Depth {detect.depth:.2f} m, Velocity {relative_velocity:.2f} m/s')
                    async_buzzer()
                    current_rot = radar_data.transform.rotation
                    fw_vec = carla.Vector3D(x=detect.depth - 0.25)
                    world_transform = carla.Transform(
                        carla.Location(),
                        carla.Rotation(
                            pitch=current_rot.pitch + math.degrees(detect.altitude),
                            yaw=current_rot.yaw + azi,
                            roll=current_rot.roll))
                    point_location = radar_data.transform.location + world_transform.transform(fw_vec)
                    self.debug.draw_point(
                        point_location,
                        size=0.075,
                        life_time=0.06,
                        persistent_lines=False,
                        color=carla.Color(255, 0, 0))

    def destroy(self):
        for sensor in self.sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()

# ==============================================================================
# -- UltrasonicSensorSystem ----------------------------------------------------
# ==============================================================================

class UltrasonicSensorSystem(object):
    def __init__(self, parent_actor, hud):
        self.sensors = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        self.debug = world.debug
        self.last_detection_time = {}
        self.distance_measurements = {}
        
        '''------communication variables------'''
        self.uart_enabled = False
        self.command_thread_active = False
        self.command_thread = None
        self.last_uart_transmission = 0
        self.uart_transmission_interval = 0.05 
        self.auto_parking_active = False
        self.last_command_time = 0
        self.waiting_for_ack = False
        self.last_data_sent_time = 0
        
        '''------ultrasonic sensor variables------'''
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', '20') 
        bp.set_attribute('vertical_fov', '5')
        bp.set_attribute('range', '9.0')

        self.sensor_positions = [
            # Front sensors (4)
            (carla.Transform(carla.Location(x=2.5, y=-0.2, z=0.5), carla.Rotation(yaw=0)), 'Front Center1'),
            (carla.Transform(carla.Location(x=2.5, y=0.2, z=0.5), carla.Rotation(yaw=0)), 'Front Center2'),
            (carla.Transform(carla.Location(x=2.5, y=-0.8, z=0.5), carla.Rotation(yaw=-30)), 'Front Far Left'),
            (carla.Transform(carla.Location(x=2.5, y=0.8, z=0.5), carla.Rotation(yaw=30)), 'Front Far Right'),
            
            # Rear sensors (4)
            (carla.Transform(carla.Location(x=-2.5, y=-0.2, z=0.5), carla.Rotation(yaw=180)), 'Rear Center1'),
            (carla.Transform(carla.Location(x=-2.5, y=0.2, z=0.5), carla.Rotation(yaw=180)), 'Rear Center2'),
            (carla.Transform(carla.Location(x=-2.5, y=-0.8, z=0.5), carla.Rotation(yaw=210)), 'Rear Far Left'),
            (carla.Transform(carla.Location(x=-2.5, y=0.8, z=0.5), carla.Rotation(yaw=150)), 'Rear Far Right'),
            
            # Left side sensors (2)
            (carla.Transform(carla.Location(x=2.3, y=-0.9, z=0.5), carla.Rotation(yaw=-90)), 'Left Front'),
            (carla.Transform(carla.Location(x=-2.3, y=-0.9, z=0.5), carla.Rotation(yaw=-90)), 'Left Rear'),
            
            # Right side sensors (2)
            (carla.Transform(carla.Location(x=2.3, y=0.9, z=0.5), carla.Rotation(yaw=90)), 'Right Front'),
            (carla.Transform(carla.Location(x=-2.3, y=0.9, z=0.5), carla.Rotation(yaw=90)), 'Right Rear')
        ]
        
        # attach the sensors to the vehicle
        for transform, sensor_name in self.sensor_positions:
            radar_sensor = world.spawn_actor(bp, transform, attach_to=self._parent)
            radar_sensor.listen(lambda radar_data, pos=sensor_name: self._ultrasonic_callback(radar_data, pos))
            self.sensors.append(radar_sensor)
            self.distance_measurements[sensor_name] = float('inf')
    
        self.draw_debug_lines = True
        world.on_tick(self._on_world_tick)
        self.last_tick_time = 0
        self.debug_lines = []
        
        '''------UART communication setup------'''
        try:
            try:
                self.uart = SerialPort('COM5', 115200, timeout=0.1)
                self.uart_enabled = True
                print("UART connection established")
                
                # Start a thread to receive commands from TivaC
                self.command_thread_active = True
                self.command_thread = threading.Thread(target=self._receive_commands)
                self.command_thread.daemon = True
                self.command_thread.start()
            except Exception as e:
                print(f"UART connection failed: {e}. Running without hardware communication.")
                self.uart_enabled = False
                self.command_thread = None 
        except NameError:
            print("SerialPort class not available. Please check your PySerial installation.")
            self.uart_enabled = False
            self.command_thread = None
        self.last_uart_transmission = 0
        self.uart_transmission_interval = 0.02  
    

    '''
    * Service Name: __ultrasonic_callback__
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): radar_data, sensor_name
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to process the radar data and check for objects in the vicinity of the vehicle
    *              and filter the readings to reduce noise and stabilize the measurements to help it 
    *              detect objects more accurately and verify it in harsh conditions then send the data to TivaC using 
    *              UART serial communication
    '''
    def _ultrasonic_callback(self, radar_data, sensor_name):
        closest_distance = float('inf')
        for detection in radar_data:
            if detection.depth >= 0.05:  
                closest_distance = min(closest_distance, detection.depth)
        
        if not hasattr(self, 'sensor_history'):
            self.sensor_history = {name: [] for name, _ in self.sensor_positions}
            

        ''' Filter the distance measurements'''
        '''
        how is the filtering done?
        this filter is based on a moving median filter
        it keeps track of the last 5 measurements for each sensor
        and calculates the median of those values
        if all values are infinite, it returns infinity
        if the sensor has not been seen before, it initializes the history with the current distance
        the filtered distance is then stored in the distance_measurements dictionary that will be later 
        used to send the data to TivaC
        '''
        if sensor_name in self.sensor_history:
            history = self.sensor_history[sensor_name]
            history.append(closest_distance)
            if len(history) > 5:  
                history.pop(0)
            if all(d == float('inf') for d in history):
                filtered_distance = float('inf')
            else:
                finite_values = [d for d in history if d != float('inf')]
                if finite_values:
                    filtered_distance = sorted(finite_values)[len(finite_values)//2]
                else:
                    filtered_distance = float('inf')
        else:
            filtered_distance = closest_distance
            self.sensor_history[sensor_name] = [closest_distance]
        self.distance_measurements[sensor_name] = filtered_distance
    

    '''
    * Service Name: __on_world_tick__
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): timestamp
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to process the world tick event and update the distance measurements
    *              and check if the distance is less than 5 meters then update the persistence counter
    '''
    def _on_world_tick(self, timestamp):    
        '''
        first we check if the parent actor is alive
        if not we return
        then we check if the object_persistence attribute is set
        if not we initialize it with a dictionary of zeros
        then we iterate over the distance measurements
        and check if the distance is less than 5 meters in order to update the persistence counter
        if the distance is greater than 5 meters we decrease the persistence counter
        if the persistence counter is less than 3 we set the distance to infinity
        this is done to filter out noise and stabilize the measurements
        this is done to help the vehicle detect objects more accurately in harsh conditions
        '''        
        try:
            if self._parent is None or not hasattr(self._parent, 'is_alive') or not self._parent.is_alive:
                return
            current_time = timestamp.elapsed_seconds
            if not hasattr(self, 'object_persistence'):
                self.object_persistence = {}
                for _, name in self.sensor_positions:
                    self.object_persistence[name] = 0
            for name, distance in self.distance_measurements.items():
                if name not in self.object_persistence:
                    self.object_persistence[name] = 0
                if distance < 5.0:
                    self.object_persistence[name] += 1
                else:
                    self.object_persistence[name] = max(0, self.object_persistence[name] - 1)
            for name in list(self.distance_measurements.keys()):
                if name in self.object_persistence and self.object_persistence[name] < 3:
                    self.distance_measurements[name] = float('inf')
        except Exception as e:
            print(f"Error in ultrasonic sensor tick: {e}")

        '''
        after finishing the distance measurements
        we check if the UART communication is enabled then we send the data to TivaC
        we check if the last transmission time is greater than the transmission interval
        '''
        if (hasattr(self, 'uart_enabled') and 
            self.uart_enabled and 
            hasattr(self, 'uart') and 
            self.uart is not None and
            hasattr(self, 'auto_parking_active') and 
            self.auto_parking_active):
            
            if current_time - self.last_uart_transmission >= self.uart_transmission_interval:
                self.last_uart_transmission = current_time
                self.send_data_to_tiva()

    '''
    * Service Name: __send_data_to_tiva__
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): None
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to send the data to TivaC using UART serial communication suing connection oriented
    *              communication and send the data in a specific format and wait for an acknowledgmentto check if the data was received
    *              corretly and processed by the TivaC if the ack is not received within 1.5 seconds then it will send new frame
    *              and reset the waiting_for_ack flag
    '''
    def send_data_to_tiva(self):
        try:
            if hasattr(self, 'waiting_for_ack') and self.waiting_for_ack:
                return
            vehicle = self._parent
            speed = vehicle.get_velocity()
            speed_kmh = 3.6 * math.sqrt(speed.x**2 + speed.y**2 + speed.z**2)
            control = vehicle.get_control()
            throttle_percent = control.throttle * 100
            brake_percent = control.brake * 100
            steering = control.steer * 100 
            sensor_data_cm = []
            for name in [
                'Front Center1', 'Front Center2', 'Front Far Left', 'Front Far Right',
                'Rear Center1', 'Rear Center2', 'Rear Far Left', 'Rear Far Right',
                'Left Front', 'Left Rear', 'Right Front', 'Right Rear'
            ]:
                value = int(min(900, self.distance_measurements.get(name, float('inf')) * 100))
                sensor_data_cm.append(value)
            
            # Format as a string: SD:speed,throttle,brake,steering,s1,s2,s3,...,s12#
            # SD: means "Sensor Data"
            data_str = f"SD          :{speed_kmh:.0f},{throttle_percent:.0f},{brake_percent:.0f},{steering:.0f}"
            
            # sensor data values in string format no need to worry about format there is string library
            # in Tiva C M4f that will handle conversion 
            for s in sensor_data_cm:
                data_str += f",{s}"

            data_str += "#"

            self.uart.write(data_str.encode('ascii'))
            print(f"Sent data string: {data_str}")
            self.waiting_for_ack = True
            self.last_data_sent_time = time.time()
        except Exception as e:
            print(f"Error sending data to Tiva C: {e}")


    '''
    * Service Name: __receive_commands__
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): None
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to receive commands from TivaC using UART serial communication
    *              processing the commands and update the vehicle control accordingly
    *              and check if the command is valid and if the command is not valid. it will print an error message and ignore the command
    *              and check if the command is an ACK message then it will reset the waiting_for_ack flag
    *              this is done using a separate thread to avoid blocking the main thread and nake the system real-time as possible
    *              for farther explanation read the commented lines lines in the code to see how this part works
    '''
    def _receive_commands(self):
        print("Command receiving thread started")
        '''
        we initialize a buffer to store the incoming data
        we use a while loop to keep the thread running and check for incoming data
        '''
        buffer = ""
        

        '''
        we check if the UART communication is enabled and the UART object is not None
        if not we sleep for 0.1 seconds to avoid busy waiting
        then we check if there is data in the UART buffer
        '''
        while self.command_thread_active:
            try:
                if not hasattr(self, 'uart') or self.uart is None or not self.uart_enabled:
                    time.sleep(0.1)
                    continue
                '''
                if there is data in the UART buffer we read the data and decode it
                first we check if the buffer is empty
                if it is empty we read the data from the UART buffer
                if there is data in the buffer we read the data and decode it
                then we check if there is a '#' character in the buffer
                if there is we split the buffer into commands using the '#' character
                then we process each command and check if it is an ACK message or a command message
                if it is an ACK message we reset the waiting_for_ack flag in order make sure that the TivaC received the data
                & processed it correctly and begin the transmission of the next frame
                if it is a command message we process the command and update the vehicle control accordingly
                if it is not a command message we print the message as a debug message
                '''
                if self.uart.in_waiting > 0:
                    data = self.uart.read(self.uart.in_waiting).decode('ascii', errors='ignore')
                    buffer += data
                    while '#' in buffer:
                        cmd_end = buffer.find('#')
                        cmd_str = buffer[:cmd_end].strip()
                        buffer = buffer[cmd_end + 1:]
                        print(f"Raw command: {cmd_str}")
                        if "ACK" in cmd_str:
                            print("Received acknowledgment from TivaC")
                            self.waiting_for_ack = False
                            continue

                        if "CMD:" in cmd_str:
                            cmd_part = cmd_str[cmd_str.find("CMD:"):]
                            parts = cmd_part[4:].split(',')
                            if len(parts) == 2:
                                try:
                                    command = int(parts[0])
                                    value = int(parts[1])
                                    print(f"Received command: {command}, value: {value}")
                                    self._process_command(command, value)
                                except ValueError:
                                    print(f"Invalid command format: {cmd_str}")
                        else:
                            if cmd_str:
                                print(f"Debug from TivaC: {cmd_str}")
                if hasattr(self, 'waiting_for_ack') and self.waiting_for_ack and hasattr(self, 'last_data_sent_time'):
                    if time.time() - self.last_data_sent_time > 1.5:
                        print("ACK timeout - resetting waiting_for_ack")
                        self.waiting_for_ack = False
                    
                time.sleep(0.05)
                
            except Exception as e:
                print(f"Error in command thread: {e}")
                time.sleep(0.1)

                
    '''
    * Service Name: __process_command__
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): command_type, value
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to process the command received from TivaC and update the vehicle control accordingly
    '''
    def _process_command(self, command_type, value):
        if not self.auto_parking_active and command_type != 0: 
            self.auto_parking_active = True
            self.hud.notification("Auto-parking activated by TivaC")

        vehicle = self._parent
        control = carla.VehicleControl()
        control.manual_gear_shift = True

        '''
        this command are simply predefined commands that are sent from TivaC
        the command type is an integer that represents the command which is initialized at the beginning
        of this class above in the code in init function method.
        they are also found on COM_PROTOCOLD.h file in TivaC folder.
        '''
        
        # CONTROL_FORWARD
        if command_type == 1: 
            stopping_control = carla.VehicleControl()
            stopping_control.throttle = 0.0
            stopping_control.brake = 1.0
            stopping_control.manual_gear_shift = True
            stopping_control.gear = 0  
            vehicle.apply_control(stopping_control)
            time.sleep(0.00001) 
            control.throttle = max(0.3, value / 100.0) 
            control.brake = 0.0
            control.steer = 0.0
            control.reverse = False
            control.hand_brake = False 
            control.gear = 1
            self.hud.notification(f"Moving forward at {int(control.throttle*100)}% throttle")

        # CONTROL_STOP    
        elif command_type == 2:  
            control.throttle = 0.0
            control.brake = 1.0
            control.steer = 0.0
            control.hand_brake = True
            self.hud.notification("Stopping vehicle")

        # CONTROL_STEER_LEFT    
        elif command_type == 3:  
            control.throttle = 0.6
            control.brake = 0.0
            control.steer = -0.5
            control.reverse = False
            control.hand_brake = False
            control.gear = 1
            self.hud.notification("Steering left")

        # CONTROL_STEER_RIGHT    
        elif command_type == 4:  
            control.throttle = 0.6
            control.brake = 0.0
            control.steer = 0.5
            control.reverse = False
            control.hand_brake = False
            control.gear = 1
            self.hud.notification("Steering right")

        # CONTROL_FORWARD_LEFT    
        elif command_type == 5:  
            control.throttle = max(0.6, value / 100.0)  
            control.brake = 0.0
            control.reverse = False
            control.steer = -0.8
            control.hand_brake = False
            control.gear = 1
            self.hud.notification(f"Moving forward-left at {int(control.throttle*100)}% throttle")

        # CONTROL_FORWARD_RIGHT    
        elif command_type == 6:  
            control.throttle = max(0.6, value / 100.0)  
            control.brake = 0.0
            control.reverse = False
            control.steer = 0.8
            control.hand_brake = False
            control.gear = 1
            self.hud.notification(f"Moving forward-right at {int(control.throttle*100)}% throttle")

        # CONTROL_REVERSE_LEFT    
        elif command_type == 7:  
            stopping_control = carla.VehicleControl()
            stopping_control.throttle = 0.0
            stopping_control.brake = 1.0
            stopping_control.manual_gear_shift = True
            stopping_control.gear = 0  
            vehicle.apply_control(stopping_control)
            time.sleep(0.000001) 
            control.manual_gear_shift = True
            control.gear = -1
            control.throttle = 0.7  
            control.brake = 0.0
            control.reverse = True
            control.hand_brake = False
            control.steer = -0.7
            self.hud.notification("Reversing left")

        # CONTROL_REVERSE_RIGHT    
        elif command_type == 8:  
            stopping_control = carla.VehicleControl()
            stopping_control.throttle = 0.0
            stopping_control.brake = 1.0
            stopping_control.manual_gear_shift = True
            stopping_control.gear = 0  
            vehicle.apply_control(stopping_control)
            time.sleep(0.000001) 
            control.manual_gear_shift = True
            control.gear = -1
            control.throttle = 0.7  
            control.brake = 0.0
            control.reverse = True
            control.hand_brake = False
            control.steer = 0.8
            self.hud.notification("Reversing right")

        # CONTROL_BACKWARD    
        elif command_type == 9:  
            stopping_control = carla.VehicleControl()
            stopping_control.throttle = 0.0
            stopping_control.brake = 1.0
            stopping_control.manual_gear_shift = True
            stopping_control.gear = 0  
            vehicle.apply_control(stopping_control)
            time.sleep(0.00001) 
            control.throttle = max(0.6, value / 100.0) 
            control.brake = 0.0
            control.reverse = True
            control.manual_gear_shift = True
            control.hand_brake = False
            control.gear = -1
            control.steer = 0.0
            self.hud.notification(f"Reversing straight at {int(control.throttle*100)}% throttle")

        vehicle.apply_control(control)
        self.last_command_time = time.time()
        print(f"Applied control: throttle={control.throttle}, brake={control.brake}, steer={control.steer}, reverse={control.reverse}, gear={control.gear}, hand_brake={control.hand_brake}")
        world = vehicle.get_world()
        world.tick()

    '''
    * Service Name: __activate_auto_parking__
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): None
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to activate the auto-parking mode by sending a signal to TivaC
    '''
    def activate_auto_parking(self):
        """Send activation signal to TivaC to start auto-parking"""
        if self.uart_enabled and self.uart is not None:
            try:
                while self.uart.in_waiting > 0:
                    discard = self.uart.read(self.uart.in_waiting)
                    print(f"Discarded {len(discard)} bytes from input buffer")
                
                # Send the activation byte (0x50 'P' for start parking)
                self.uart.write(bytes([0x50]))  
                self.hud.notification("Auto-parking activation sent to TivaC")
                self.auto_parking_active = True
                print("Auto-parking activated - starting data transmission to TivaC")
                time.sleep(0.1)
                if self.uart.in_waiting > 0:
                    print(f"Received {self.uart.in_waiting} bytes after activation")
            except Exception as e:
                print(f"Error sending auto-parking activation: {e}")
        else:
            print("UART not enabled or connected - can't activate auto-parking")
    
    '''
    * Service Name: __get_distance__
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): sensor_name
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: distance
    * Description: to get the distance measurement from the specified sensor
    '''
    def get_distance(self, sensor_name):
        return self.distance_measurements.get(sensor_name, float('inf'))
    

    '''
    * Service Name: __get_all_distances__
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): None
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: distance_measurements
    * Description: to get all distance measurements from all sensors
    '''
    def get_all_distances(self):
        return self.distance_measurements.copy()

    '''
    * Service Name: __destroy__
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): None
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to destroy the ultrasonic sensor system and stop the command thread
    *              and close the UART connection if it is enabled
    '''
    def destroy(self):
        # Stop the command thread
        if hasattr(self, 'command_thread_active'):
            self.command_thread_active = False
            if hasattr(self, 'command_thread') and self.command_thread is not None and self.command_thread.is_alive():
                self.command_thread.join(timeout=1.0)
        
        # Closing the UART connection
        if hasattr(self, 'uart') and hasattr(self, 'uart_enabled') and self.uart_enabled and self.uart is not None:
            try:
                self.uart.close()
            except:
                pass

        for sensor in self.sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()

        self.sensors.clear()

# ==============================================================================
# -- EmergencyBrakingSystem ----------------------------------------------------
# ==============================================================================
#This code is written is written by Automotive Embedded Software Engineer
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
        self.brake_start_sim_time = None
        self.brake_duration_sim = 3.0 
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
        
    '''
    * Service Name: _create_radar_sensors_
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): world
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to create the radar sensors and set their attributes then attach them to the vehicle
    *             there is only 3 radars in front with custom modifications to simulate real world configurations in
    *             Automotive industry
    '''
    def _create_radar_sensors(self, world):
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
    @staticmethod
    def _safe_radar_callback(weak_self, radar_data, sensor_position):

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
    def _radar_callback(self, radar_data, sensor_position):
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
    def _calculate_collision_risk(self, ttc, distance, rel_velocity, speed):
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
    def _apply_staged_braking(self):
        # Skip if driver override is active
        if self.driver_override:
            return
        # Skip braking if insufficient detections or in cooling period
        if self.detection_counter < self.detection_threshold:
            return
        # Check for cooling period after a hard brake
        if self.brake_level <= 1 and time.time() - self.last_intervention_time < self.cooling_period:
            return
            
        # Get current simulation time
        world = self._parent.get_world()
        current_sim_time = world.get_snapshot().timestamp.elapsed_seconds
        
        # Current control for maintaining other driver inputs
        current_control = self._parent.get_control()
        
        # Apply appropriate brake force according to level
        # Light braking (30%)
        if self.brake_level == 1:  
            control = current_control
            control.throttle = 0.0
            control.brake = 0.3
            control.hand_brake = False
            self._parent.apply_control(control)
            
            if not self.brake_active:
                self.hud.notification("Brake Assist", seconds=1.0)
            self.brake_active = True
            self.override_active = False

        # Medium braking (60%)     
        elif self.brake_level == 2:
            # Start sim time tracking if not already started
            if self.brake_start_sim_time is None:
                self.brake_start_sim_time = current_sim_time
                
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 0.6
            control.hand_brake = False
            control.steer = current_control.steer * 0.5  
            control.manual_gear_shift = False
            self._parent.apply_control(control)
            
            # Check if we've applied brakes for the desired simulation time
            if current_sim_time - self.brake_start_sim_time >= self.brake_duration_sim:
                # After sim time duration, apply full stop
                control = carla.VehicleControl()
                control.throttle = 0.0
                control.brake = 1.0
                control.hand_brake = True
                control.manual_gear_shift = True
                control.gear = 0  # Neutral
                self._parent.apply_control(control)
                self.brake_start_sim_time = None  # Reset timer
                
            if not self.brake_active or self.brake_level == 1:
                self.hud.notification("BRAKE ASSIST", seconds=1.5)
                async_buzzer()
            self.brake_active = True
            self.override_active = True
            
        # Full braking (100%)
        elif self.brake_level == 3:
            # Start sim time tracking if not already started
            if self.brake_start_sim_time is None:
                self.brake_start_sim_time = current_sim_time
                
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 1.0
            control.hand_brake = True
            control.manual_gear_shift = True
            control.gear = 0  # Neutral
            self._parent.apply_control(control)
            
            # Check if we've applied brakes for the desired simulation time
            if current_sim_time - self.brake_start_sim_time >= self.brake_duration_sim:
                # After sim time duration, ensure vehicle remains stopped
                control = carla.VehicleControl()
                control.throttle = 0.0
                control.brake = 1.0
                control.hand_brake = True
                control.manual_gear_shift = True
                control.gear = 0  # Neutral
                self._parent.apply_control(control)
                self.brake_start_sim_time = None  # Reset timer
                
            if not self.brake_active or self.brake_level < 3:
                self.hud.notification("EMERGENCY BRAKE!", seconds=2.0)
                async_buzzer()
                async_buzzer()
            self.brake_active = True
            self.override_active = True
                    
        self.last_intervention_time = time.time()
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
# In the EmergencyBrakingSystem._safety_thread_function method
    def _safety_thread_function(self):
        while self.safety_thread_active:
            try:
                if self.brake_level == 3 and self.override_active:
                    # Get current simulation time
                    world = self._parent.get_world()
                    current_sim_time = world.get_snapshot().timestamp.elapsed_seconds
                    
                    velocity = self._parent.get_velocity()
                    speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                    
                    # If simulation time is being tracked, continue applying brakes
                    if self.brake_start_sim_time is not None:
                        # Apply brakes while within the simulation time window
                        if current_sim_time - self.brake_start_sim_time < self.brake_duration_sim:
                            control = carla.VehicleControl()
                            control.throttle = 0.0
                            control.brake = 1.0
                            control.manual_gear_shift = True
                            control.reverse = False
                            control.hand_brake = True
                            control.gear = 0 
                            self._parent.apply_control(control)
                            if self.debug:
                                print(f"[EBS] Emergency brake applied: level 3, sim time: {current_sim_time - self.brake_start_sim_time:.2f}/{self.brake_duration_sim}s")
                        else:
                            # After duration, keep the vehicle stopped
                            self.brake_start_sim_time = None
                            
                    # Always ensure the vehicle is stopped after emergency braking
                    if speed_kmh > 0.5:
                        control = carla.VehicleControl()
                        control.throttle = 0.0
                        control.brake = 1.0
                        control.manual_gear_shift = True
                        control.reverse = False
                        control.hand_brake = True
                        control.gear = 0 
                        self._parent.apply_control(control)
                        
                elif self.brake_level == 2 and self.override_active:
                    # Similar logic for medium braking
                    world = self._parent.get_world()
                    current_sim_time = world.get_snapshot().timestamp.elapsed_seconds
                    
                    if self.brake_start_sim_time is not None:
                        if current_sim_time - self.brake_start_sim_time < self.brake_duration_sim:
                            control = carla.VehicleControl()
                            control.throttle = 0.0
                            control.brake = 0.6
                            control.manual_gear_shift = True
                            control.reverse = False
                            control.hand_brake = False
                            control.gear = 0
                            self._parent.apply_control(control)
                        else:
                            self.brake_start_sim_time = None
                            self.brake_level = 0
                            self.override_active = False
                else:
                    pass
                    
                time.sleep(0.01)  # Small sleep to avoid busy waiting
                
            except Exception as e:
                print(f"[EBS] Safety thread error: {e}")
                time.sleep(0.5)

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
    def destroy(self):
        if hasattr(self, 'safety_thread_active'):
            self.safety_thread_active = False
        if hasattr(self, 'safety_thread') and self.safety_thread is not None:
            self.safety_thread.join(timeout=1.0)

        for sensor in self.sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        self.sensors = []
# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================
#This code is written is written by Automotive Embedded Software Engineer

class CameraManager(object):
    '''
    * Service Name: __CameraManager__
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): parent_actor, hud, gamma_correction
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to initialize the camera manager and set its attributes
    *              Creating the camera sensors and set their attributes
    '''
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        self.front_camera_sensor = None
        self.lane_detection_initialized = False
        self.left_lane_history = []
        self.right_lane_history = []
        self.history_length = 15
        self.current_status = None
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        Attachment = carla.AttachmentType
        if not self._parent.type_id.startswith("walker.pedestrian"):
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.0*bound_x, y=+0.0*bound_y, z=2.0*bound_z), carla.Rotation(pitch=8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=+0.8*bound_x, y=+0.0*bound_y, z=1.3*bound_z)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=+1.9*bound_x, y=+1.0*bound_y, z=1.2*bound_z)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-2.8*bound_x, y=+0.0*bound_y, z=4.6*bound_z), carla.Rotation(pitch=6.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-1.0, y=-1.0*bound_y, z=0.4*bound_z)), Attachment.Rigid)]
        else:
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=2.5, y=0.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-4.0, z=2.0), carla.Rotation(pitch=6.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=0, y=-2.5, z=-0.0), carla.Rotation(yaw=90.0)), Attachment.Rigid)]
        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)', {}],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)', {}],
            ['sensor.camera.instance_segmentation', cc.CityScapesPalette, 'Camera Instance Segmentation (CityScapes Palette)', {}],
            ['sensor.camera.instance_segmentation', cc.Raw, 'Camera Instance Segmentation (Raw)', {}],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {'range': '50'}],
            ['sensor.camera.dvs', cc.Raw, 'Dynamic Vision Sensor', {}],
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
                {'lens_circle_multiplier': '3.0',
                'lens_circle_falloff': '3.0',
                'chromatic_aberration_intensity': '0.5',
                'chromatic_aberration_offset': '0'}],
            ['sensor.camera.optical_flow', cc.Raw, 'Optical Flow', {}],
            ['sensor.camera.normals', cc.Raw, 'Camera Normals', {}],
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            elif item[0].startswith('sensor.lidar'):
                self.lidar_range = 50

                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
                    if attr_name == 'range':
                        self.lidar_range = float(attr_value)
            item.append(bp)
        self.index = None
        try:
            model_path = 'd:/CarlaSim/PythonAPI/examples/GP_modules/LeNETModel.h5'
            self.traffic_sign_detector = TrafficSignDetector(
                model_path=model_path, 
                confidence_threshold=0.75, 
                min_detections=3, 
                time_window=0.8, 
                process_every_n_frames=2
            )
            print("Traffic sign detector initialized")
            self.traffic_signs_detected = {}
        except Exception as e:
            print(f"Failed to initialize traffic sign detector: {e}")
            self.traffic_sign_detector = None
        self.add_front_camera_sensor()


    '''
    * Service Name: add_front_camera_sensor
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): None
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to add a front camera sensor to the vehicle and set its attributes
    '''
    def add_front_camera_sensor(self):
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        camera_bp = bp_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '640')
        camera_bp.set_attribute('image_size_y', '480')
        camera_bp.set_attribute('fov', '90')
        camera_transform = carla.Transform(carla.Location(x=2.5, z=2.0))  # Adjust the location as needed
        self.front_camera_sensor = world.spawn_actor(camera_bp, camera_transform, attach_to=self._parent)
        self.front_camera_sensor.listen(lambda image: self.process_front_camera_image(image))

    '''
    * Service Name: initialize_lane_detection
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): None
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to initialize lane detection and set its attributes and start the lane departure warning thread
    '''
    def initialize_lane_detection(self):
        self.lane_detection_initialized = True
        self.left_lane_history = []
        self.right_lane_history = []
        self.history_length = 15
        self.current_status = None
        self.rightcross = False
        self.leftcross = False
        self.ldw_thread_active = False
        self.ldw_thread = None
        self.lane_data_queue = queue.Queue(maxsize=3)
        self.lane_state_lock = threading.Lock()
        self.deviation = 0
        self.crossing_state = {"left": False, "right": False, "center": True}
        self.ldw_thread_active = True
        self.ldw_thread = threading.Thread(target=self._lane_departure_warning_thread, daemon=True)
        self.ldw_thread.start()
        print("Lane departure warning thread started")
    '''
    * Service Name: process_front_camera_image
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): image
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to process the front camera image and check for traffic signs
    *              and update the lane detection status
    '''
    def process_front_camera_image(self, image):
        if not self.lane_detection_initialized:
            self.initialize_lane_detection()
        array = np.array(image.raw_data).reshape((image.height, image.width, 4))[:, :, :3]
        processed_frame = array.copy() 
        # Process the image for traffic sign detection
        if hasattr(self, 'traffic_sign_detector') and self.traffic_sign_detector:
            try:
                processed_frame = self.traffic_sign_detector.process_frame(array)
            except Exception as e:
                print(f"Error in traffic sign detection: {e}")
        
        self.front_surface = pygame.surfarray.make_surface(processed_frame.swapaxes(0, 1))
        self._process_lane_detection(array.copy())

    '''
    * Service Name: _process_lane_detection
    * Sync/Async: Synchronous
    * Reentrancy: Non-Reentrant
    * Parameters (in): image
    * Parameters (inout): None
    * Parameters (out): None
    * Return value: None
    * Description: to process the lane detection and check the lane crossing state and skipping in auto-parking mode
    '''
    def _process_lane_detection(self, image):
        # Skip in auto-parking mode
        if hasattr(self._parent, 'get_world') and hasattr(self._parent.get_world(), 'ultrasonic_sensors') and \
            hasattr(self._parent.get_world().ultrasonic_sensors, 'auto_parking_active') and \
            self._parent.get_world().ultrasonic_sensors.auto_parking_active:
                # Create parking mode message on the image
                height, width = image.shape[:2]
                frame = image.copy()
                cv2.putText(frame, "AUTO PARKING ACTIVE", (width//2-150, height//2), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self.lane_surface = pygame.surfarray.make_surface(frame.swapaxes(0, 1))
                return
        frame = image.copy()
        height, width = frame.shape[:2]
        
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
        def lanes_detection(img):
            gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            blurred_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
            edges = cv2.Canny(blurred_img, 50, 150, apertureSize=3)
            height, width = img.shape[:2]
            roi_vertices = [(0, height), (width // 2, height // 2), (width, height)]
            roi_mask = np.zeros_like(edges)
            cv2.fillPoly(roi_mask, np.array([roi_vertices], dtype=np.int32), 255)
            masked_edges = cv2.bitwise_and(edges, roi_mask)
            lines = cv2.HoughLinesP(masked_edges, rho=1, theta=np.pi/180, threshold=50,
                                    minLineLength=20, maxLineGap=300)
            left_lines, right_lines = [], []
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    slope = (y2 - y1) / (x2 - x1 + 1e-6)
                    if slope < -0.5:
                        left_lines.append(line)
                    elif slope > 0.5:
                        right_lines.append(line)

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
            def average_line(lines):
                if not lines:
                    return [0, 0, 0, 0]
                x1 = np.mean([line[0][0] for line in lines])
                y1 = np.mean([line[0][1] for line in lines])
                x2 = np.mean([line[0][2] for line in lines])
                y2 = np.mean([line[0][3] for line in lines])
                return [int(x1), int(y1), int(x2), int(y2)]
              
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
        def extend_line(line, img_height):
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
            
            # Package lane data for the warning thread
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
    def _lane_departure_warning_thread(self):
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
    def _create_singleton_lka_thread(self, deviation, world_object):
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
        
        # Creating new thread
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

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            scaled_surface = pygame.transform.scale(self.surface, (800, 600))
            display.blit(self.surface, (0, 0))

        if hasattr(self, 'front_surface') and self.front_surface is not None:
            display.blit(self.front_surface, (self.hud.dim[0], 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / (2.0 * self.lidar_range)
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        elif self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        elif self.sensors[self.index][0].startswith('sensor.camera.optical_flow'):
            image = image.get_color_coded_flow()
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)
        if hasattr(self._parent, 'get_world') and hasattr(self._parent.get_world(), 'ultrasonic_sensors') and \
        hasattr(self._parent.get_world().ultrasonic_sensors, 'auto_parking_active') and \
        self._parent.get_world().ultrasonic_sensors.auto_parking_active:
            array = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            cv2.putText(array, "AUTO PARKING MODE", (image.width//2-150, image.height//2), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            return

# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2000.0)

        sim_world = client.get_world()
        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.033
            sim_world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)

        main_width = 800 
        main_height = 600
        lane_width = 640 
        lane_height = 480  

        # Calculate total window dimensions
        total_width = main_width + lane_width
        total_height = max(main_height, lane_height)
        
        display = pygame.display.set_mode(
            (total_width, total_height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        
        # Create surfaces for each view
        main_surface = pygame.Surface((main_width, main_height))
        lane_surface = pygame.Surface((lane_width, lane_height))
        
        # Create HUD for main view
        hud = HUD(main_width, main_height)
        world = World(sim_world, hud, args)
        controller = KeyboardControl(world, args.autopilot)

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        clock = pygame.time.Clock()
        while True:
            if args.sync:
                sim_world.tick()
            clock.tick_busy_loop(30) 

            if controller.parse_events(client, world, clock, args.sync):
                return
            world.tick(clock)

            main_surface.fill((0, 0, 0))
            lane_surface.fill((0, 0, 0))
            world.camera_manager.render(main_surface)
            world.hud.render(main_surface)
            if hasattr(world.camera_manager, 'lane_surface') and world.camera_manager.lane_surface is not None:
                scaled_lane = pygame.transform.scale(world.camera_manager.lane_surface, (lane_width, lane_height))
                lane_surface.blit(scaled_lane, (0, 0))
            display.blit(main_surface, (0, 0))
            display.blit(lane_surface, (main_width, 0))
            pygame.display.flip()

    finally:
        if original_settings:
            sim_world.apply_settings(original_settings)
        if world and world.recording_enabled:
            client.stop_recorder()
        if world is not None:
            world.destroy()
        pygame.quit()

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1270x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()