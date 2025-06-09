# V2V Communication System

A comprehensive Vehicle-to-Vehicle (V2V) communication simulation system built with OMNeT++, INET Framework, Veins, and SUMO for accident detection and emergency warning protocols.

## 🚗 Overview

This project implements a realistic V2V communication network that enables vehicles to automatically detect accidents and broadcast emergency warnings to nearby vehicles in real-time. The system uses IEEE 802.11p DSRC protocol for wireless communication and integrates traffic simulation with network simulation for comprehensive testing.

## 🎯 Key Features

- **Real-Time Accident Detection**: Automatic accident scenario generation at predefined simulation times
- **Emergency Message Broadcasting**: Immediate V2V communication of accident warnings
- **Intelligent Message Relay**: Smart relay mechanisms with distance and probability filtering
- **Visual Simulation**: Color-coded vehicle states (red for accidents, green for message received)
- **Comprehensive Logging**: Detailed accident and communication event logging
- **Anti-Flooding Protection**: Duplicate message detection and collision avoidance
- **Route Redirection**: Automatic vehicle rerouting towards accident locations

## 🏗️ System Architecture

### Core Components

```
V2V System
├── Application Layer
│   ├── VeinsInetSampleApplication - Main V2V application
│   ├── AccidentCreator - Accident scenario generator
│   └── ApplicationLifecycle - Application management
├── Communication Layer
│   ├── PacketProcessor - Message processing and validation
│   ├── PacketSender - Message transmission
│   ├── MessageValidator - Message integrity checking
│   └── RelayManager - Intelligent relay logic
├── Utility Layer
│   ├── LogHandler - Event logging system
│   ├── MessageIdGenerator - Unique ID generation
│   └── VehicleDisplayManager - Visual state management
└── Integration Layer
    ├── VeinsInetMobility - Vehicle movement
    ├── VeinsInetManager - SUMO integration
    └── VeinsInetManagerForker - Process management
```

## 🛠️ Technology Stack

- **Simulation Framework**: OMNeT++ 5.6+
- **Network Framework**: INET Framework 4.x
- **Vehicular Framework**: Veins 5.2
- **Traffic Simulator**: SUMO 1.8+
- **Communication Protocol**: IEEE 802.11p DSRC
- **Programming Language**: C++14/17
- **Operating System**: Windows 10/11

## 📋 Prerequisites

Before running the simulation, ensure you have the following installed:

1. **OMNeT++ 5.6 or later**
   - Download from: https://omnetpp.org/
   - Add to system PATH

2. **INET Framework 4.x**
   - Download from: https://inet.omnetpp.org/
   - Extract to `F:/inet4/` (or update Makefile paths)

3. **Veins 5.2**
   - Download from: https://veins.car2x.org/
   - Extract to `F:/veins-veins-5.2/` (or update Makefile paths)

4. **SUMO 1.8+**
   - Download from: https://sumo.dlr.de/
   - Add `sumo-gui.exe` to system PATH

## 🚀 Installation & Setup

> **Important**: All building, compilation, and simulation operations are performed in **OMNeT++ IDE**, not VS Code or other editors. VS Code is used only for code viewing and documentation.

### 1. Open Project in OMNeT++ IDE
1. Launch **OMNeT++ IDE** from your installation directory
2. Create new workspace or use existing one
3. Import the V2V project: `File → Import → General → Existing Projects into Workspace`
4. Select the `G:\University\GP\V2V` directory

### 2. Configure Framework Dependencies
In OMNeT++ IDE:
1. Right-click project → `Properties`
2. Go to `Project References`
3. Check the following dependencies:
   - **INET Framework** (inet4)
   - **Veins Framework** (veins-5.2)

### 3. Build the Project in OMNeT++
In OMNeT++ IDE:
1. Right-click on the project → `Build Project`
2. Or use the build button in the toolbar
3. For release build: Set build configuration to `release`
4. For debug build: Set build configuration to `debug`

### 4. Verify Build
Check that `libV2V.dll` is created in the `src/` directory.

## 🎮 Running the Simulation

### Quick Start
```powershell
# Navigate to simulations directory
cd simulations\V2V_simulation

# Run with OMNeT++ GUI
omnetpp

# Or run from command line
..\run -u Cmdenv -c General -r 0 omnetpp.ini
```

### Advanced Run Options
```powershell
# Run with specific configuration
..\run -u Qtenv -c V2VAccidentScenario omnetpp.ini

# Run in command line mode (faster)
..\run -u Cmdenv -c General omnetpp.ini

# Run with custom parameters
..\run -u Qtenv --sim-time-limit=60s omnetpp.ini
```

## 📊 Simulation Scenarios

### Default Accident Scenario
The simulation includes three predefined accident scenarios:

| Vehicle | Accident Time | Duration | Resume Time |
|---------|---------------|----------|-------------|
| host[0] | t=20s | 30s | t=50s |
| host[2] | t=35s | 25s | t=60s |
| host[4] | t=50s | 20s | t=70s |

### Simulation Timeline
- **0-20s**: Normal traffic flow
- **20s**: First accident (vehicle[0] stops, broadcasts emergency)
- **35s**: Second accident (vehicle[2] stops, broadcasts emergency)
- **50s**: Third accident (vehicle[4] stops, broadcasts emergency)
- **60s**: Simulation ends

## 📁 Project Structure

```
V2V/
├── README.md                          # This file
├── V2V_System_Documentation.txt       # Complete system documentation
├── simulations/                       # Simulation configurations
│   ├── omnetpp.ini                   # Global simulation settings
│   ├── run                           # Execution script
│   └── V2V_simulation/               # Main simulation scenario
│       ├── map.sumo.cfg             # SUMO configuration
│       ├── map.net.xml              # Road network
│       ├── map.rou.xml              # Vehicle routes
│       ├── Scenario.ned             # Network topology
│       └── V2V_accident_messages.txt # Simulation logs
└── src/                              # Source code
    ├── Makefile                      # Build configuration
    ├── libV2V.dll                   # Compiled library
    └── veins_inet/                   # Core implementation
        ├── VeinsInetSampleApplication.* # Main V2V application
        ├── AccidentCreator.*         # Accident scenario management
        ├── PacketProcessor.*         # Message processing
        ├── PacketSender.*           # Message transmission
        ├── MessageValidator.*       # Message validation
        ├── RelayManager.*           # Intelligent relay logic
        ├── LogHandler.*             # Logging system
        ├── MessageIdGenerator.*     # Unique ID generation
        ├── VehicleDisplayManager.*  # Visual management
        └── VeinsInetSampleMessage.* # Message definitions
```

## 🔧 Configuration

### Key Parameters (omnetpp.ini)
```ini
# Simulation duration
sim-time-limit = 60s

# Number of vehicles
*.numNodes = 10

# Communication range
*.node[*].wlan[0].radio.transmitter.power = 23dBm

# Update interval
*.manager.updateInterval = 0.1s

# SUMO configuration
*.manager.launchConfig = xmldoc("map.launchd.xml")
```

### Message Structure
Each V2V message contains:
- **Car ID**: Unique vehicle identifier
- **Road ID**: Current road segment
- **Accident Coordinates**: GPS location (X, Y)
- **Timestamp**: Message creation time

## 📊 Output & Analysis

### Log Files
- **V2V_accident_messages.txt**: Detailed communication logs
- **Plain result files**: Statistical simulation data

### Visual Indicators
- **Red vehicles**: Currently in accident state
- **Green vehicles**: Received emergency messages
- **Blue vehicles**: Normal operation state

### Performance Metrics
- Message propagation delay
- Network coverage percentage
- Relay efficiency
- Duplicate message ratio

## 🧪 Testing Scenarios

### 1. Basic Functionality Test
```powershell
# Run 60-second simulation with 5 vehicles
..\run -u Qtenv -c BasicTest omnetpp.ini
```

### 2. Dense Network Test
```powershell
# Test with 50 vehicles
..\run -u Cmdenv -c DenseTraffic omnetpp.ini
```

### 3. Extended Duration Test
```powershell
# Run 300-second simulation
..\run -u Cmdenv --sim-time-limit=300s omnetpp.ini
```

## 🔍 Troubleshooting

### Common Issues

#### Build Errors
```powershell
# Clean and rebuild
make clean
make MODE=release
```

#### SUMO Connection Issues
- Verify SUMO is installed and in PATH
- Check `map.launchd.xml` configuration
- Ensure SUMO port (9999) is available

#### Missing Dependencies
- Verify INET_PROJ and VEINS_PROJ paths in Makefile
- Check OMNeT++ installation
- Ensure all required DLLs are accessible

### Debug Mode
```powershell
# Build in debug mode
make MODE=debug

# Run with detailed logging
..\run -u Qtenv -f omnetpp.ini -c Debug
```

## 📈 Performance Optimization

### For Large Simulations
1. **Use Command Line Mode**: `-u Cmdenv` for faster execution
2. **Disable GUI Updates**: Set update interval to higher values
3. **Reduce Logging**: Minimize console output in release mode
4. **Optimize Relay Probability**: Adjust relay filters in RelayManager

### Memory Management
- Monitor log file sizes (auto-cleanup recommended)
- Use appropriate simulation time limits
- Consider garbage collection for message tracking

## 🔮 Future Enhancements

### Planned Features
- [ ] Machine learning-based accident prediction
- [ ] 5G-V2X protocol integration
- [ ] Multi-hop routing algorithms
- [ ] Real-world map integration
- [ ] Security and authentication mechanisms
- [ ] Dynamic vehicle density adaptation

### Research Extensions
- [ ] Cooperative perception algorithms
- [ ] Edge computing integration
- [ ] Blockchain-based trust mechanisms
- [ ] Energy-efficient communication protocols

## 🤝 Contributing

### Development Guidelines
1. Follow existing code documentation format
2. Add function documentation for all public methods
3. Update README.md for new features
4. Test with multiple vehicle densities
5. Maintain backward compatibility

### Code Style
- Use descriptive variable names
- Add comprehensive comments
- Follow C++ best practices
- Document all public interfaces

## 📄 License

This project is developed for academic research purposes. Please cite appropriately if used in publications.


## 🏆 Acknowledgments

This project integrates several open-source frameworks:
- **OMNeT++**: Discrete event simulation framework
- **INET**: Internet simulation framework
- **Veins**: Vehicular network simulation framework
- **SUMO**: Traffic simulation platform
