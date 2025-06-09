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
"""

"""
this file is modified by OptiDrive. The team is group of Automotive Embedded Software Engineers and Machine Learing Engineers

"""

# from __future__ import print_function


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

# Update your import section
import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
import struct
import winsound
import threading
import time
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
    from pygame.locals import K_F4
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')



# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================

def play_buzzer():
    # Play a 500Hz sound for 100ms
    winsound.Beep(500, 100)

def async_buzzer():
    # Create a thread to play the buzzer sound asynchronously
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
        self.dms = None
        self.dms_blinker_active = False
        self.auto_parking = AutoParking(self.player, self.hud, self.ultrasonic_sensors)
        self.auto_parking.init()
        

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
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get the specific blueprint for Lincoln MKZ 2020.
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
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])

        # Spawn the player.
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
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.ultrasonic_sensors = UltrasonicSensorSystem(self.player, self.hud)
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
        if self.dms is not None:
            try:
                self.dms.update()
                
                # Check if driver is sleeping and control blinker
                if self.dms.is_sleeping():
                    if not self.dms_blinker_active:
                        # Activate right blinker
                        if isinstance(self.player, carla.Vehicle):
                            # Direct assignment of enum value instead of bitwise operations
                            self.player.set_light_state(carla.VehicleLightState.RightBlinker)
                            self.dms_blinker_active = True
                            self.hud.notification('Driver sleeping detected - Right blinker activated')
                            # Play alert sound
                            import winsound
                            winsound.Beep(2500, 200)
                else:
                    if self.dms_blinker_active:
                        # Turn off all lights if it was activated by DMS
                        if isinstance(self.player, carla.Vehicle):
                            self.player.set_light_state(carla.VehicleLightState.NONE)
                            self.dms_blinker_active = False
                            self.hud.notification('Driver alert - Blinker deactivated')
            except Exception as e:
                print(f"Error in DMS update: {e}")
        # Rest of the tick method...
        if (self.ultrasonic_sensors and 
            hasattr(self.ultrasonic_sensors, 'auto_parking_active') and 
            self.ultrasonic_sensors.auto_parking_active):
            # Just check if we've received a command recently
            current_time = time.time()
            if (hasattr(self.ultrasonic_sensors, 'last_command_time') and 
                current_time - self.ultrasonic_sensors.last_command_time < 0.5):
                # We've received a command recently, but DON'T return
                # Let the world tick normally below
                pass
        
        # This code will now run during auto-parking
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

            if self.auto_parking.is_active():
                # Get ultrasonic sensor readings
                sensor_readings = []
                distances = self.ultrasonic_sensors.get_all_distances()
                
                # Extract values in the required order
                for name in [
                    'Front Center1', 'Front Center2', 'Front Far Left', 'Front Far Right',
                    'Rear Center1', 'Rear Center2', 'Rear Far Left', 'Rear Far Right',
                    'Left Front', 'Left Rear', 'Right Front', 'Right Rear'
                ]:
                    # Convert to centimeters and cap at 900cm
                    value = min(900, distances.get(name, float('inf')) * 100)
                    sensor_readings.append(int(value))
                
                # Get vehicle speed
                velocity = self.player.get_velocity()
                speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                
                # Process auto-parking logic
                self.auto_parking.process(sensor_readings, int(speed_kmh))
    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

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
        self._F4_pressed = False
        
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
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYDOWN:
                if event.key == K_F3 and not self._F3_pressed:
                    self._F3_pressed = True
                    if not world.auto_parking.is_active():
                        world.auto_parking.activate_parallel()
                        world.hud.notification("Auto-parking activated")
                    else:
                        world.auto_parking.deactivate()
                        world.hud.notification("Auto-parking deactivated")
                elif event.key == K_F4 and not self._F4_pressed:
                    self._F4_pressed = True
                    if not world.auto_parking.is_active():
                        world.auto_parking.activate_perpendicular()
                        world.hud.notification("Perpendicular auto-parking activated")
                    else:
                        world.auto_parking.deactivate()
                        world.hud.notification("Auto-parking deactivated")
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
                elif event.key == K_F4:
                    self._F4_pressed = False
                elif event.key == K_F1:
                    world.hud.toggle_info()
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
                        world.player.enable_constant_velocity(carla.Vector3D(17, 0, 0))
                        world.constant_velocity_enabled = True
                        world.hud.notification("Enabled Constant Velocity Mode at 60 km/h")
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
            '']
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
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
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
    def __init__(self, parent_actor, hud):
        self.sensors = []
        self._parent = parent_actor
        self.hud = hud
        self.velocity_range = 7.5  # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', '45')  # Adjust FOV to cover a cone shape
        bp.set_attribute('vertical_fov', '5')
        bp.set_attribute('range', '10')
        
        # New sensor positions: one at each mirror, pointing backward.
        self.radar_positions = [
            (carla.Transform(carla.Location(x=-1.5, y=-0.7, z=1.5), carla.Rotation(yaw=180)), 'Left Mirror'),
            (carla.Transform(carla.Location(x=-1.5, y=0.7, z=1.5), carla.Rotation(yaw=180)), 'Right Mirror')
        ]
        
        for transform, position in self.radar_positions:
            sensor = world.spawn_actor(bp, transform, attach_to=self._parent)
            sensor.listen(lambda radar_data, pos=position: self._Radar_callback(radar_data, pos))
            self.sensors.append(sensor)

    def _Radar_callback(self, radar_data, position):
        # Get the car's velocity
        car_velocity = self._parent.get_velocity()
        car_speed = math.sqrt(car_velocity.x**2 + car_velocity.y**2 + car_velocity.z**2)
        
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            if abs(azi) <= 30 and detect.depth > 1.0:  # Ignore detections too close to the car
                relative_velocity = detect.velocity - car_speed
                if relative_velocity < -1.9:  # Check if the object is moving towards the car
                    self.hud.notification(f'Vehicle approaching from {position}!')
                    print(f'Vehicle approaching from {position}: Depth {detect.depth:.2f} m, Velocity {relative_velocity:.2f} m/s')
                    # Draw a red point for visualization.
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
        
        # Initialize communication variables
        self.uart_enabled = False
        self.uart = None
        self.command_thread_active = False
        self.command_thread = None
        self.last_uart_transmission = 0
        self.uart_transmission_interval = 0.05  # 50ms (20Hz transmission rate)
        self.auto_parking_active = False
        self.last_command_time = 0
        self.waiting_for_ack = False
        self.last_data_sent_time = 0
        
        # Configure radar to behave like ultrasonic sensors
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', '20')  # Narrow FOV like ultrasonic
        bp.set_attribute('vertical_fov', '5')
        bp.set_attribute('range', '9.0')  # 6 meter max range
        bp.set_attribute('points_per_second', '1500')  # Increase precision

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
        
        # Spawn the sensors
        for transform, sensor_name in self.sensor_positions:
            radar_sensor = world.spawn_actor(bp, transform, attach_to=self._parent)
            radar_sensor.listen(lambda radar_data, pos=sensor_name: self._ultrasonic_callback(radar_data, pos))
            self.sensors.append(radar_sensor)
            self.distance_measurements[sensor_name] = float('inf')
        
        # Draw debug visualization
        self.draw_debug_lines = True
        world.on_tick(self._on_world_tick)
        self.last_tick_time = 0
        self.debug_lines = []
        
        try:
            try:
                self.uart = SerialPort('COM5', 9600, timeout=0.1)
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
                self.command_thread = None  # Explicitly set to None
        except NameError:
            print("SerialPort class not available. Please check your PySerial installation.")
            self.uart_enabled = False
            self.command_thread = None  # Explicitly set to None
        
        # Setup data transmission timer
        self.last_uart_transmission = 0
        self.uart_transmission_interval = 0.05  # 50ms (20Hz transmission rate)
        
    def _ultrasonic_callback(self, radar_data, sensor_name):
        closest_distance = float('inf')
        has_detection = False
        
        # Find the closest detection point
        detection_count = 0
        for detection in radar_data:
            detection_count += 1
            if detection.depth >= 0.03:  # Reduced minimum to 3cm
                has_detection = True
                closest_distance = min(closest_distance, detection.depth)
        
        # If no detection or empty radar data, explicitly set to maximum range
        if not has_detection:
            closest_distance = 9.0  # Use the full 9 meter range when nothing is detected
        
        
        # Rest of the method remains the same
        if not hasattr(self, 'sensor_history'):
            self.sensor_history = {name: [] for name, _ in self.sensor_positions}
            
        # Update history
        if sensor_name in self.sensor_history:
            history = self.sensor_history[sensor_name]
            history.append(closest_distance)
            # Reduce history length for faster response
            if len(history) > 3:
                history.pop(0)
            
            # Use weighted average instead of median - more weight to recent readings
            if all(d >= 8.9 for d in history):  # Consider anything ≥ 8.9m as max range
                filtered_distance = 9.0
            else:
                # Only consider valid detections (under max range)
                valid_values = [d for d in history if d < 8.9]
                if valid_values:
                    # Apply weighting as before
                    if len(valid_values) >= 3:
                        weights = [0.2, 0.3, 0.5]
                        filtered_distance = sum(v * w for v, w in zip(valid_values[-3:], weights))
                    elif len(valid_values) == 2:
                        filtered_distance = valid_values[0] * 0.4 + valid_values[1] * 0.6
                    else:
                        filtered_distance = valid_values[0]
                else:
                    filtered_distance = 9.0  # No valid detections = max range
        else:
            filtered_distance = closest_distance
            self.sensor_history[sensor_name] = [closest_distance]
        
        # Save the filtered measurement
        self.distance_measurements[sensor_name] = filtered_distance
    def _on_world_tick(self, timestamp):            
        try:
            # Check if parent actor is still valid before proceeding
            if self._parent is None or not hasattr(self._parent, 'is_alive') or not self._parent.is_alive:
                return
                    
            current_time = timestamp.elapsed_seconds
            
            # Make sure object_persistence is initialized properly at the beginning
            if not hasattr(self, 'object_persistence'):
                self.object_persistence = {}
                for _, name in self.sensor_positions:
                    self.object_persistence[name] = 0
            
            # Update sensor visualizations at 10Hz
            if current_time - self.last_tick_time >= 0.1:
                self.last_tick_time = current_time
        
            for name, distance in self.distance_measurements.items():
                # Make sure the key exists in object_persistence
                if name not in self.object_persistence:
                    self.object_persistence[name] = 0
                    
                # Update the persistence counter with increased sensitivity
                if distance < 9.0:  
                    self.object_persistence[name] += 1
                else:
                    self.object_persistence[name] = max(0, self.object_persistence[name] - 1)

            # Reduce persistence threshold for quicker response
            for name in list(self.distance_measurements.keys()):
                if name in self.object_persistence and self.object_persistence[name] < 2:  # Reduced from 3 to 2
                    self.distance_measurements[name] = float('inf')
        except Exception as e:
            print(f"Error in ultrasonic sensor tick: {e}")
            
        # Only send data if UART is enabled AND auto-parking is active or we're in diagnostic mode
        if (hasattr(self, 'uart_enabled') and 
            self.uart_enabled and 
            hasattr(self, 'uart') and 
            self.uart is not None and
            hasattr(self, 'auto_parking_active') and 
            self.auto_parking_active):
            
            if current_time - self.last_uart_transmission >= self.uart_transmission_interval:
                self.last_uart_transmission = current_time
                self.send_data_to_tiva()

    def send_data_to_tiva(self):
        try:
            # Check if we're waiting for an ACK
            if hasattr(self, 'waiting_for_ack') and self.waiting_for_ack:
                # If waiting for ACK, don't send more data
                return
                
            # Get vehicle data
            vehicle = self._parent
            speed = vehicle.get_velocity()
            speed_kmh = 3.6 * math.sqrt(speed.x**2 + speed.y**2 + speed.z**2)
            control = vehicle.get_control()
            throttle_percent = control.throttle * 100
            brake_percent = control.brake * 100
            steering = control.steer * 100 
            
            # Collect sensor data
            sensor_data_cm = []
            for name in [
                'Front Center1', 'Front Center2', 'Front Far Left', 'Front Far Right',
                'Rear Center1', 'Rear Center2', 'Rear Far Left', 'Rear Far Right',
                'Left Front', 'Left Rear', 'Right Front', 'Right Rear'
            ]:
                # Cap at 900cm and convert to int
                value = int(min(900, self.distance_measurements.get(name, float('inf')) * 100))
                sensor_data_cm.append(value)
            
            # Format as a string: SD:speed,throttle,brake,steering,s1,s2,s3,...,s12#
            # SD: means "Sensor Data"
            data_str = f"SD         :{speed_kmh:.0f},{throttle_percent:.0f},{brake_percent:.0f},{steering:.0f}"
            
            # Add sensor values
            for s in sensor_data_cm:
                data_str += f",{s}"
                
            # Add terminating character for TivaC string parsing
            data_str += "#"
            
            # Send the string
            self.uart.write(data_str.encode('ascii'))
            print(f"Sent data string: {data_str}")
            
            # Set flag that we're waiting for acknowledgment
            self.waiting_for_ack = True
            self.last_data_sent_time = time.time()
            
            # No need for sleep - we'll wait for the ACK
            
        except Exception as e:
            print(f"Error sending data to Tiva C: {e}")

    def _receive_commands(self):
        """Receive string commands from TivaC"""
        print("Command receiving thread started")
        buffer = ""
        
        while self.command_thread_active:
            try:
                if not hasattr(self, 'uart') or self.uart is None or not self.uart_enabled:
                    time.sleep(0.1)
                    continue
                    
                # Check if data is available
                if self.uart.in_waiting > 0:
                    # Read available data
                    data = self.uart.read(self.uart.in_waiting).decode('ascii', errors='ignore')
                    buffer += data
                    
                    # Process complete commands (ending with #)
                    while '#' in buffer:
                        cmd_end = buffer.find('#')
                        cmd_str = buffer[:cmd_end].strip()
                        buffer = buffer[cmd_end + 1:]
                        
                        # Debug the raw command string
                        print(f"Raw command: {cmd_str}")
                        
                        # Check for ACK message
                        if "ACK" in cmd_str:
                            print("Received acknowledgment from TivaC")
                            self.waiting_for_ack = False
                            continue
                        
                        # Extract CMD: portion regardless of whether it's part of a debug message
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
                            # Debug output
                            if cmd_str:
                                print(f"Debug from TivaC: {cmd_str}")
                    
                # Add a timeout mechanism for waiting for ACK
                if hasattr(self, 'waiting_for_ack') and self.waiting_for_ack and hasattr(self, 'last_data_sent_time'):
                    if time.time() - self.last_data_sent_time > 1.5:  # 1.5 second timeout
                        print("ACK timeout - resetting waiting_for_ack")
                        self.waiting_for_ack = False
                    
                time.sleep(0.05)  # Short sleep
                
            except Exception as e:
                print(f"Error in command thread: {e}")
                time.sleep(0.1)
    def _process_command(self, command_type, value):
        """Process a command received from TivaC"""
        if not self.auto_parking_active and command_type != 0: 
            self.auto_parking_active = True
            self.hud.notification("Auto-parking activated by TivaC")

        
        # Get current vehicle control
        vehicle = self._parent
        control = vehicle.get_control()
        
        # Process command based on type
        if command_type == 1:  # CONTROL_FORWARD
            control.throttle = max(0.3, value / 100.0)  # Value is throttle percentage
            control.brake = 0.0
            control.steer = 0.0
            control.reverse = False

            self.hud.notification(f"Moving forward at {value}% throttle")
            
        elif command_type == 2:  # CONTROL_STOP
            control.throttle = 0.0
            control.brake = 1.0
            control.steer = 0.0
            self.hud.notification("Stopping vehicle")
            
        elif command_type == 3:  # CONTROL_STEER_LEFT
            control.throttle = 0.2
            control.brake = 0.0
            control.steer = -0.5  # Turn left
            control.reverse = False
            self.hud.notification("Steering left")
            
        elif command_type == 4:  # CONTROL_STEER_RIGHT
            control.throttle = 0.2
            control.brake = 0.0
            control.steer = 0.5  # Turn right
            control.reverse = False
            self.hud.notification("Steering right")
            
        elif command_type == 5:  # CONTROL_FORWARD_LEFT
            control.throttle = min(1.0, value / 100.0)  # Value is throttle percentage
            control.brake = 0.0
            control.reverse = False
            control.steer = -0.5  # Turn left
            self.hud.notification(f"Moving forward-left at {value}% throttle")
        
        elif command_type == 6:  # CONTROL_FORWARD_RIGHT
            control.throttle = min(1.0, value / 100.0)  # Value is throttle percentage
            control.brake = 0.0
            control.reverse = False
            control.steer = 0.7  # Turn right
            #make the speed at const value as the value is throttle percentage
            self.hud.notification(f"Moving forward-right at {value}% throttle")
        
        elif command_type == 7:  # CONTROL_REVERSE_LEFT
            control.throttle = min(1.0, value / 100.0)  # Value is throttle percentage
            control.brake = 0.0
            control.reverse = True
            control.steer = -0.7  # Turn left
            self.hud.notification(f"Reversing left at {value}% throttle")
        
        elif command_type == 8:  # CONTROL_REVERSE_RIGHT
            control.throttle = min(1.0, value / 100.0)  # Value is throttle percentage
            control.brake = 0.0
            control.reverse = True
            control.steer = 0.7  # Turn right
            self.hud.notification(f"Reversing right at {value}% throttle")

        vehicle.apply_control(control)
        print(f"Applied control: throttle={control.throttle}, brake={control.brake}, steer={control.steer}, reverse={control.reverse}")
        try:
            vehicle.get_world().tick()
        except:
            pass


    def activate_auto_parking(self):
        """Send activation signal to TivaC to start auto-parking"""
        if self.uart_enabled and self.uart is not None:
            try:
                # Clear any buffered data first
                while self.uart.in_waiting > 0:
                    discard = self.uart.read(self.uart.in_waiting)
                    print(f"Discarded {len(discard)} bytes from input buffer")
                
                # Send the activation byte (0x50 'P' for start parking)
                self.uart.write(bytes([0x50]))  # 0x50 is ASCII 'P'
                self.hud.notification("Auto-parking activation sent to TivaC")
                self.auto_parking_active = True
                print("Auto-parking activated - starting data transmission to TivaC")
                
                # Wait briefly for any immediate response
                time.sleep(0.1)
                if self.uart.in_waiting > 0:
                    print(f"Received {self.uart.in_waiting} bytes after activation")
            except Exception as e:
                print(f"Error sending auto-parking activation: {e}")
        else:
            print("UART not enabled or connected - can't activate auto-parking")
    
    def get_distance(self, sensor_name):
        return self.distance_measurements.get(sensor_name, float('inf'))
    
    def get_all_distances(self):
        return self.distance_measurements.copy()

    def destroy(self):
        # Stop the command thread
        if hasattr(self, 'command_thread_active'):
            self.command_thread_active = False
            if hasattr(self, 'command_thread') and self.command_thread is not None and self.command_thread.is_alive():
                self.command_thread.join(timeout=1.0)
        
        # Close UART
        if hasattr(self, 'uart') and hasattr(self, 'uart_enabled') and self.uart_enabled and self.uart is not None:
            try:
                self.uart.close()
            except:
                pass
                
        # Destroy sensors
        for sensor in self.sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        self.sensors.clear()

# Add this class before the game_loop() function

# ==============================================================================
# -- AutoParking ---------------------------------------------------------------
# ==============================================================================

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
    def __init__(self, parent_actor, hud, ultrasonic_system):
        self.parent_actor = parent_actor
        self.hud = hud
        self.ultrasonic_system = ultrasonic_system
        self.parking_status = self.PARKING_STATE_IDLE
        self.parking_active = False
        self.debug_mode = True 
        self.simulation_time = 0.0
        self.parking_mode = "none"
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
    def init(self):
        self.parking_status = self.PARKING_STATE_IDLE
        self.parking_active = False
        self.hud.notification("Auto-parking system initialized")
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
    def activate_parallel(self):
        self.parking_status = self.PARKING_STATE_IDLE
        self.parking_active = True
        self.hud.notification("Auto-parking activated")
        
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
    def activate(self):
        self.activate_parallel()

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
    def activate_perpendicular(self):
        self.parking_status = self.PARKING_STATE_PERP_SEARCHING
        self.parking_active = True
        self.parking_mode = "perpendicular"
        self.hud.notification("Perpendicular auto-parking activated")
        print("Starting perpendicular parking search...")
        if hasattr(self, 'perp_phase'):
            self.perp_phase = 0
    
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
    def deactivate(self):
        self.parking_status = self.PARKING_STATE_IDLE
        self.parking_active = False
        self.send_data(self.CONTROL_STOP, 0)
        self.hud.notification("Auto-parking deactivated")
    
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
    def is_active(self):
        return self.parking_active
    
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
    def get_state(self):
        return self.parking_status
    
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
    def process(self, sensor_readings, vehicle_speed):
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
    def process_perpendicular_parking(self, sensor_readings, vehicle_speed):

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


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


import cv2
import numpy as np

class CameraManager(object):
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

        # Add the front camera sensor
        self.add_front_camera_sensor()

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

    def process_front_camera_image(self, image):
        if not self.lane_detection_initialized:
            self.initialize_lane_detection()
        
        # Convert image to numpy array
        array = np.array(image.raw_data).reshape((image.height, image.width, 4))[:, :, :3]
        # Create pygame surface
        self.front_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        
        # Process lanes
        self.detect_lanes(array)

    def initialize_lane_detection(self):
        self.lane_detection_initialized = True
        self.left_lane_history = []
        self.right_lane_history = []
        self.history_length = 15
        self.current_status = None
        self.rightcross = False
        self.leftcross = False



    def detect_lanes(self, image):
        # Convert the image to a numpy array
        frame = image.copy()
        
        # Lane detection function based on the new code
        def lanes_detection(img):
            # Convert the image to grayscale
            gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            # Apply Gaussian blur
            blurred_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
            # Apply Canny edge detection
            edges = cv2.Canny(blurred_img, 50, 150, apertureSize=3)
            # Define region of interest
            height, width = img.shape[:2]
            roi_vertices = [(0, height), (width // 2, height // 2), (width, height)]
            roi_mask = np.zeros_like(edges)
            # Fill the ROI polygon
            cv2.fillPoly(roi_mask, np.array([roi_vertices], dtype=np.int32), 255)
            # Mask the edges image
            masked_edges = cv2.bitwise_and(edges, roi_mask)
            # Apply Hough Line Transform
            lines = cv2.HoughLinesP(masked_edges, rho=1, theta=np.pi/180, threshold=50,
                                    minLineLength=20, maxLineGap=300)
            # Separate the lines into left and right lanes
            left_lines, right_lines = [], []
            # If no lines are detected, return None
            if lines is not None:
                # Iterate over the lines and classify them as left or right
                for line in lines:
                    # Extract the coordinates of the line
                    x1, y1, x2, y2 = line[0]
                    # Calculate the slope of the line
                    slope = (y2 - y1) / (x2 - x1 + 1e-6) # Small epsilon to avoid division by zero
                    # Classify the line based on the slope
                    if slope < -0.5:
                        # Left lane
                        left_lines.append(line)
                    elif slope > 0.5:
                        # Right lane
                        right_lines.append(line)

            # Function to calculate the average line to help smooth the output
            #and fill gaps in the detection
            def average_line(lines):
                if not lines:
                    return [0, 0, 0, 0]
                # Calculate the average x and y coordinates
                x1 = np.mean([line[0][0] for line in lines])
                y1 = np.mean([line[0][1] for line in lines])
                x2 = np.mean([line[0][2] for line in lines])
                y2 = np.mean([line[0][3] for line in lines])
                return [int(x1), int(y1), int(x2), int(y2)]
                # Calculate the average line for left and right lanes
            left_line = average_line(left_lines)
            right_line = average_line(right_lines)
            return left_line, right_line, (height, width)

        left_line, right_line, (height, width) = lanes_detection(frame)
        
        # Use lane history to fill gaps if the current detection is missing.
        # This is a simple way to smooth the output.
        if left_line == [0, 0, 0, 0]:
            # If the current detection is missing, use the history
            if self.left_lane_history:
                # Use the last known left lane line
                # This is a simple way to smooth the output
                left_line = self.left_lane_history[-1]
        else:
            # If the current detection is valid, update the history
            self.left_lane_history.append(left_line)
            # Keep the history length fixed
            if len(self.left_lane_history) > self.history_length:
                # Remove the oldest entry   
                self.left_lane_history.pop(0)
        
        if right_line == [0, 0, 0, 0]:
            # If the current detection is missing, use the history
            if self.right_lane_history:
                # Use the last known right lane line
                right_line = self.right_lane_history[-1]
        else:
            # If the current detection is valid, update the history
            self.right_lane_history.append(right_line)
            # Keep the history length fixed
            if len(self.right_lane_history) > self.history_length:
                # Remove the oldest entry
                self.right_lane_history.pop(0)

        # Function to extend a line based on its slope and intercept.
        def extend_line(line, img_height):
            # If the line is missing, return a dummy line
            if line == [0, 0, 0, 0]:
                return line
                # Extract the line coordinates
            x1, y1, x2, y2 = line # Unpack the line coordinates
            # Calculate the slope and intercept of the line
            slope = (y2 - y1) / (x2 - x1 + 1e-6) # Small epsilon to avoid division by zero
            # Calculate the y-intercept
            intercept = y1 - slope * x1
            # Extend down to the bottom of the image...
            y_bottom = img_height
            x_bottom = int((y_bottom - intercept) / slope)
            y_top = int(img_height * 0.6)
            x_top = int((y_top - intercept) / slope)
            return [x_bottom, y_bottom, x_top, y_top]
            # Extend the left and right lines
        left_line = extend_line(left_line, height)
        right_line = extend_line(right_line, height)
        
        thickness = 9  # Line thickness
        if left_line != [0, 0, 0, 0]:
            cv2.line(frame, (left_line[0], left_line[1]), (left_line[2], left_line[3]), (255, 0, 0), thickness)
        if right_line != [0, 0, 0, 0]:
            cv2.line(frame, (right_line[0], right_line[1]), (right_line[2], right_line[3]), (255, 0, 0), thickness)
        # Calculate the deviation from the center of the lane
        # The deviation is the difference between the center of the image and the center of the lane
        # The center of the lane is the average of the x-coordinates of the left and right lines
        car_position = width // 2 # Assume the car is at the center of the image
        
        if left_line != [0, 0, 0, 0] and right_line != [0, 0, 0, 0]:
            # Both lines are detected
            # Calculate the lane center at the bottom of the image (where the car is)
            lane_center = (left_line[0] + right_line[0]) // 2
            deviation = car_position - lane_center
            threshold = width // 8
            margin = threshold // 3  # Increased hysteresis margin
            
            # Draw lane center line for visualization
            cv2.line(frame, (lane_center, height), (lane_center, height-50), (0, 255, 0), 2)
            
            # Add a persistence counter to avoid rapid state changes
            if not hasattr(self, 'crossing_persistence'):
                self.crossing_persistence = {'right': 0, 'left': 0, 'center': 0}
            
            # Determine vehicle position
            if deviation > threshold + margin:
                # Car is deviating to the right
                self.crossing_persistence['right'] += 1
                self.crossing_persistence['left'] = 0
                self.crossing_persistence['center'] = 0
            elif deviation < -threshold - margin:
                # Car is deviating to the left
                self.crossing_persistence['left'] += 1
                self.crossing_persistence['right'] = 0
                self.crossing_persistence['center'] = 0
            else:
                # Car is within the lane
                self.crossing_persistence['center'] += 1
                self.crossing_persistence['right'] = 0
                self.crossing_persistence['left'] = 0
            
            # Only change state when persistent for multiple frames
            persistence_threshold = 3
            
            if self.crossing_persistence['right'] >= persistence_threshold:
                new_status = "Car is crossing the right lane"
                self.rightcross = True
                self.leftcross = False
                # If the right blinker is not active, activate the buzzer
                if not (self._parent.get_light_state() & carla.VehicleLightState.RightBlinker):
                    async_buzzer()
            elif self.crossing_persistence['left'] >= persistence_threshold:
                new_status = "Car is crossing the left lane"
                self.rightcross = False
                self.leftcross = True
                # If the left blinker is not active, activate the buzzer
                if not (self._parent.get_light_state() & carla.VehicleLightState.LeftBlinker):
                    async_buzzer()
            elif self.crossing_persistence['center'] >= persistence_threshold:
                new_status = "Car is within the lane"
                self.rightcross = False
                self.leftcross = False
            else:
                # Keep current status while we're not sure
                new_status = self.current_status
                
            # Add visual indicator for debugging
            if self.rightcross:
                cv2.putText(frame, "RIGHT CROSS", (width-150, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            elif self.leftcross:
                cv2.putText(frame, "LEFT CROSS", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                cv2.putText(frame, "CENTER", (width//2-40, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            if new_status != self.current_status:
                # print(new_status)
                self.current_status = new_status

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.lane_surface = pygame.surfarray.make_surface(frame.swapaxes(0, 1))


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
        
        # Render front camera and lane detection side by side
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

        # Create main window with space for both views
        main_width = 800  # Control view width
        main_height = 600  # Control view height
        lane_width = 640  # Lane detection width
        lane_height = 480  # Lane detection height

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
            clock.tick_busy_loop(30)  # Reduce to 30 FPS for better performance

            if controller.parse_events(client, world, clock, args.sync):
                return

            # Update world and get camera feeds
            world.tick(clock)
            
            # Clear surfaces
            main_surface.fill((0, 0, 0))
            lane_surface.fill((0, 0, 0))
            
            # Render main view to its surface
            world.camera_manager.render(main_surface)
            world.hud.render(main_surface)
            
            # Render lane detection to its surface if available
            if hasattr(world.camera_manager, 'lane_surface') and world.camera_manager.lane_surface is not None:
                scaled_lane = pygame.transform.scale(world.camera_manager.lane_surface, (lane_width, lane_height))
                lane_surface.blit(scaled_lane, (0, 0))
            
            # Blit both surfaces to the main display
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
