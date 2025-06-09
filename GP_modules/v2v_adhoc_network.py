import carla
import random
import time
import datetime
import math
import numpy as np
import json
from enum import Enum

# it is enum for priority of messages
class MessagePriority(Enum):
    LOW = 0
    MEDIUM = 1
    HIGH = 2
    EMERGENCY = 3
# it is enum for type of messages
class MessageType(Enum):
    DRIVER_EMERGENCY = 0    
    COLLISION = 1         
    EMERGENCY_BRAKING = 2  
    HAZARD_WARNING = 3      
    LANE_CHANGE = 4        
    VEHICLE_STATUS = 5      
    TRAFFIC_INFO = 6       


class V2VMessage:

    def __init__(self, sender_id, msg_type, content, priority=MessagePriority.MEDIUM, 
                 location=None, timestamp=None):
        '''
        * Service Name: _V2VMessage
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): sender_id, msg_type, content, priority, location, timestamp
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: to initialize a V2VMessage object with the given parameters.
        '''        
        self.id = f"{sender_id}_{int(time.time()*1000)}_{random.randint(0,1000)}"
        self.sender_id = sender_id
        self.type = msg_type
        self.content = content
        self.priority = priority
        self.location = location
        self.timestamp = timestamp or datetime.datetime.now()
        self.ttl = self._get_ttl(priority)
        self.hops = 0 



    def _get_ttl(self, priority):
        '''
        * Service Name: _get_ttl
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): priority
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: ttl
        * Description: to get the time-to-live (TTL) value based on the message priority.
        *              The TTL value is used to determine how long the message should be considered valid.
        '''
        if priority == MessagePriority.EMERGENCY:
            return 60
        elif priority == MessagePriority.HIGH:
            return 30
        elif priority == MessagePriority.MEDIUM:
            return 15
        elif priority == MessagePriority.LOW:
            return 5
        else:
            raise ValueError("Invalid message priority")
    

    def to_dict(self):
        '''
        * Service Name: to_dict
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): None
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: dict
        * Description: to convert the V2VMessage object to a dictionary representation.
        '''
        return {
            'id': self.id,
            'sender_id': self.sender_id,
            'type': self.type.name,
            'priority': self.priority.name,
            'content': self.content,
            'timestamp': self.timestamp.strftime("%Y-%m-%d %H:%M:%S.%f"),
            'location': None if self.location is None else {
                'x': self.location.x, 
                'y': self.location.y,
                'z': self.location.z
            },
            'ttl': self.ttl,
            'hops': self.hops
        }
    

class V2VAdHocNetwork:

    def __init__(self, world, max_range=300.0, packet_loss_rate=0.1):
        '''
        * Service Name: _V2VAdHocNetwork
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): world, max_range, packet_loss_rate
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: to initialize a V2VAdHocNetwork object with the given parameters.
        '''
        self.world = world
        self.max_range = max_range
        self.packet_loss_rate = packet_loss_rate
        self.active_vehicles = set()
        self.vehicle_positions = {}
        self.messages = []
        self.messages_history = []
        self.last_cleanup = time.time()
        self.debug = world.debug
        self.visual_elements = {}
        self.stats = {
            'total_messages_sent': 0,
            'total_messages_received': 0,
            'total_messages_dropped': 0,
            'active_vehicles': 0
        }

    def broadcast_message(self, message):
        '''
        * Service Name: broadcast_message
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): message
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: bool    
        * Description: to broadcast a message to the V2V network.
        '''
        if random.random() < self.packet_loss_rate:
            self.stats['total_messages_dropped'] += 1
            print(f"V2V:Message {message.id} dropped due to packet loss.")
            return False
        self.messages.append(message)
        self.stats['total_messages_sent'] += 1
        self.messages_history.append(message)
        self.active_vehicles.add(message.sender_id)
        self.stats['active_vehicles'] = len(self.active_vehicles)

        if message.location:
            self.vehicle_positions[message.sender_id] = message.location
        
        if message.priority in [MessagePriority.EMERGENCY, MessagePriority.HIGH]:
            print(f"V2V EMERGENCY: [{message.type.name}] from Vehicle {message.sender_id}")
            details = message.content.get('details', {})
            reason = message.content.get('reason', 'Unknown')
            print(f"    Reason: {reason}")
            print(f"    Location: ({message.location.x:.1f}, {message.location.y:.1f})")
            print(f"    Priority: {message.priority.name}")
        else:
            print(f"V2V: {message.type.name} from {message.sender_id} - {message.content.get('summary', '')}")
        return True
    

    def recieve_messages_for_vehicle(self, vehicle_id, position):
        '''
        * Service Name: recieve_messages_for_vehicle
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): vehicle_id, position
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: relevant_messages
        * Description: to receive messages for a specific vehicle based on its position.
        '''
        relevant_messages = []
        self.vehicle_positions[vehicle_id] = position
        self.active_vehicles.add(vehicle_id)
        self.stats['active_vehicles'] = len(self.active_vehicles)
        if not hasattr(self, 'message_reception_map'):
            self.message_reception_map = {}
    
        for message in self.messages:
            if message.sender_id == vehicle_id:
                continue
            if not message.location:
                continue
            distance = math.sqrt(
                (position.x - message.location.x) ** 2 +(position.y - message.location.y) ** 2 )
            if abs(message.location.z - position.z) > 20.0:
                continue    
            if distance <= self.max_range:
                message.hops += 1
                relevant_messages.append(message)
                self.stats['total_messages_received'] += 1

                key = f"{message.sender_id}-{message.id}"
                if key not in self.message_reception_map:
                    self.message_reception_map[key] = set()
                self.message_reception_map[key].add(vehicle_id)
            
        return relevant_messages
    


    def update(self):
        '''
        * Service Name: update
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): None
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: to update the V2V network by cleaning up old messages.
        '''
        current_time = time.time()
        if current_time - self.last_cleanup > 1.0:
            self._cleanup_old_messages()
            self.last_cleanup = current_time

    def _cleanup_old_messages(self):
        '''
        * Service Name: _cleanup_old_messages
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): None
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: to clean up old messages that have expired based on their TTL.
        *              This method removes messages that are no longer valid from the network.
        '''
        current_time = datetime.datetime.now()
        active_messages = []
        
        for msg in self.messages:
            expiration = msg.timestamp + datetime.timedelta(seconds=msg.ttl)
            if current_time < expiration:
                active_messages.append(msg)

        self.messages = active_messages


    def visualize_v2v_range(self, vehicle, duration=0.1):
        '''
        * Service Name: visualize_v2v_range
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): vehicle, duration
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: to visualize the V2V range of a vehicle in the simulation.
        '''
        if not vehicle:
            return
            
        if random.random() < 0.1:
            location = vehicle.get_location()
            color = carla.Color(0, 255, 255, 50)
            segments = 12
            
            for i in range(segments):
                angle1 = 2 * math.pi * i / segments
                angle2 = 2 * math.pi * (i + 1) / segments
                x1 = location.x + self.max_range * math.cos(angle1)
                y1 = location.y + self.max_range * math.sin(angle1)
                x2 = location.x + self.max_range * math.cos(angle2)
                y2 = location.y + self.max_range * math.sin(angle2)
                start = carla.Location(x=x1, y=y1, z=location.z)
                end = carla.Location(x=x2, y=y2, z=location.z)
                self.debug.draw_line(
                    start,
                    end,
                    thickness=0.1,
                    color=color,
                    life_time=duration,
                    persistent_lines=False
                )

    def visualize_message_broadcast(self, message, duration=2.0):
        '''
        * Service Name: visualize_message_broadcast
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): message, duration
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: to visualize the broadcast of a message in the simulation.
        '''
        if not message.location:
            return
        if message.priority == MessagePriority.EMERGENCY:
            # Red
            color = carla.Color(255, 0, 0)  
        elif message.priority == MessagePriority.HIGH:
            # Orange
            color = carla.Color(255, 165, 0)
        elif message.priority == MessagePriority.MEDIUM:
            # Yellow
            color = carla.Color(255, 255, 0) 
        else:
            color = carla.Color(0, 255, 255)  

        self.visual_elements[message.id] = {
            'start_time': time.time(),
            'center': message.location,
            'color': color,
            'max_range': self.max_range,
            'duration': duration
        }

    def update_visualizations(self):
        '''
        * Service Name: update_visualizations
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): None
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: None
        * Description: to update the visualizations of the V2V network.
        '''
        current_time = time.time()
        expired_elements = []

        for msg_id, visual in self.visual_elements.items():
            elapsed = current_time - visual['start_time']
            if elapsed > visual['duration']:
                expired_elements.append(msg_id)
                continue

            progress = elapsed / visual['duration']
            radius = progress * visual['max_range']
            segments = 12 
            center = visual['center']
            color = visual['color']
            
            for i in range(segments):
                angle1 = 2 * math.pi * i / segments
                angle2 = 2 * math.pi * (i + 1) / segments
                x1 = center.x + radius * math.cos(angle1)
                y1 = center.y + radius * math.sin(angle1)
                x2 = center.x + radius * math.cos(angle2)
                y2 = center.y + radius * math.sin(angle2)
                start = carla.Location(x=x1, y=y1, z=center.z)
                end = carla.Location(x=x2, y=y2, z=center.z)
                
                self.debug.draw_line(
                    start,
                    end,
                    thickness=0.1,
                    color=color,
                    life_time=0.1,
                    persistent_lines=False
                )
        
        for msg_id in expired_elements:
            del self.visual_elements[msg_id]

    def create_emergency_message(self, vehicle, reason, priority=MessagePriority.EMERGENCY, details=None):
        '''
        * Service Name: create_emergency_message
        * Sync/Async: Synchronous
        * Reentrancy: Non-Reentrant
        * Parameters (in): vehicle, reason, priority, details
        * Parameters (inout): None
        * Parameters (out): None
        * Return value: message
        * Description: to create an emergency message for a vehicle.
        *              The message contains information about the vehicle's speed, heading, and other details.
        '''
        if not vehicle:
            return None
            
        vehicle_id = vehicle.id
        location = vehicle.get_location()
        transform = vehicle.get_transform()
        velocity = vehicle.get_velocity()
        speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        content = {
            "reason": reason,
            "speed": speed_kmh,
            "heading": transform.rotation.yaw,
            "summary": f"{reason} at {speed_kmh:.1f} km/h",
            "vehicle_type": vehicle.type_id,
            "details": details or {}
        }
        message = V2VMessage(
            sender_id=vehicle_id,
            msg_type=MessageType.DRIVER_EMERGENCY if "driver" in reason.lower() else MessageType.HAZARD_WARNING,
            content=content,
            priority=priority,
            location=location
        )
        
        return message