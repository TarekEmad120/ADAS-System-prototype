# CARLA Manual Control Lane & Parking

This project provides advanced manual control scripts for the CARLA simulator, focusing on lane keeping, emergency braking, and auto-parking systems. It is designed for research and development in autonomous driving, ADAS, and automotive embedded systems.

## Features

- **Manual Vehicle Control**: Drive vehicles in CARLA using keyboard input (WASD/arrows, space, etc.).
- **Emergency Braking System (EBS)**: Multi-stage radar-based collision avoidance with visual and acoustic warnings, staged braking, and driver override logic.
- **Lane Departure Warning (LDW)**: it is used to detect lane boundaries and provide warnings to the driver.
- **Lane Keeping Assist (LKA)**: Automatically adjusts steering to keep the vehicle within lane boundaries.
- **Blind Spot Detection(BSD)**: Monitors blind spots using radar sensors and provides warnings to the driver.
- **Auto Parking System**: Parallel and perpendicular parking using ultrasonic sensors, with state machine logic for searching, aligning, and maneuvering.
- **Sensor Integration**: Utilizes radar and ultrasonic sensors for real-time environment perception.
- **Driver Monitoring System (DMS)**: (If enabled) Monitors driver state and integrates with vehicle control.
- **Telemetry and HUD**: Real-time vehicle telemetry, notifications, and visual feedback.

## File Overview

- `manual_control_lane.py`: Main script for manual driving with all ADAS features and parallel parking logic using tiva C board.
- `manual_control_parking2.py`: Main script for manual driving with advanced auto-parking logic.

## Requirements

- **CARLA Simulator** (Tested on 0.9.15)
- **Python 3.7.9**
- **Dependencies**:
  - `pygame`
  - `numpy`
  - `pyserial` (for serial communication)
  - `carla` Python API


Install dependencies with:

```bash
pip install pygame numpy pyserial
```

## Usage

1. **Start CARLA Simulator**
   - Run `CarlaUE4.exe` or start the server as usual.

2. **Run Manual Control Script**
   - For all features including lane keeping and emergency braking:
     ```bash
     python manual_control_lane.py
     ```
   - For auto-parking:
     ```bash
     python manual_control_parking2.py
     ```

3. **Controls**
   - See the script header for a full list of keyboard controls (WASD/arrows, P for autopilot, etc.).

## Advanced Features

- **Emergency Braking**: Multi-stage logic with warnings and staged braking based on radar data.
- **Auto Parking**: State machine for searching, aligning, and parking using ultrasonic sensors.
- **Driver Override**: System detects and respects driver intervention.
- **Telemetry**: Toggle vehicle telemetry and HUD for real-time feedback.

## Customization

- Modify sensor parameters, thresholds, and logic in the respective classes (`EmergencyBrakingSystem`, `AutoParking`, etc.) to suit your research needs.
- Integrate additional modules (e.g., DMS, traffic sign recognition) by placing them in `GP_modules` and updating import paths.

## System Requirements

To run the CARLA simulator and these advanced control scripts, your system should meet or exceed the following specifications:

- **Processor**: Intel Core i7-11800H (8 cores, 16 threads @ 2.3GHz base, 4.6GHz boost)
- **RAM**: 16GB DDR4-3200 MHz
- **Graphics Card**: NVIDIA RTX 3070 8GB VRAM
- **Storage**: 512GB SSD
- **Operating System**: Windows 11 Pro 64-bit

## External Hardware Integration

- **Tiva C TM4C123GH6PM**: Required for testing parallel auto-parking in simulation. The Tiva C board communicates with CARLA by sending parking commands and receiving sensor data (ultrasonic, radar) in real time. This enables hardware-in-the-loop (HIL) testing of embedded parking algorithms.


## Dataset
- **Eye Dataset**: Used for driver monitoring system (DMS) training and evaluation.
- link to dataset: [Eye Dataset](https://www.kaggle.com/datasets/tauilabdelilah/mrl-eye-dataset)

## Credits

Developed by OptiDrive: A team of Automotive Embedded Software Engineers and Machine Learning Engineers.

Based on the original CARLA manual control example, with significant enhancements for ADAS and autonomous driving research.
