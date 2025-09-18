# ERob Motor Controller Library

A comprehensive C++ library for controlling EtherCAT servo motors using the SOEM (Simple Open EtherCAT Master) library. This library provides high-level control interfaces for robotic applications, specifically designed for three-axis robot arm control.

## Features

### Core Functionality
- **Automatic Motor Discovery**: Automatically scans and configures all connected EtherCAT servo motors
- **Real-time Control**: 1ms control cycle with real-time trajectory planning
- **Thread-safe Operations**: Multi-threaded architecture with proper synchronization
- **Error Handling**: Comprehensive error reporting and recovery mechanisms
- **State Management**: Full DS402 state machine implementation

### Motor Control Modes
- **Profile Position Mode**: Point-to-point movements with configurable velocity and acceleration
- **Servo Control Mode**: Real-time position tracking with smooth trajectory planning
- **Cyclic Synchronous Position**: 1ms position updates for precise control
- **Emergency Stop**: Immediate motor shutdown functionality

### Advanced Features
- **Multi-motor Coordination**: Simultaneous control of multiple axes
- **Motion Planning**: Quintic polynomial trajectory generation
- **Position Tolerance**: Configurable precision for position reaching
- **Encoder Conversion**: Automatic counts-to-degrees conversion
- **Homing Support**: Reference position finding for each axis

## Quick Start

### Installation

1. Clone the repository:
```bash
git clone https://github.com/yulaomao/eRob_SOEM_linux.git
cd eRob_SOEM_linux
```

2. Build the library:
```bash
mkdir build && cd build
cmake ..
make -j4
```

3. Install (optional):
```bash
sudo make install
```

### Basic Usage

```cpp
#include "ERobMotorController.hpp"

int main() {
    // Create controller instance
    ERobMotorController controller("enp6s0", 1000); // 1ms cycle time
    
    // Initialize and scan for motors
    auto result = controller.initialize();
    if (result != ControllerError::SUCCESS) {
        std::cerr << "Failed to initialize: " 
                  << ERobMotorController::getErrorString(result) << std::endl;
        return -1;
    }
    
    // Enable motor 1
    controller.enableMotor(1);
    
    // Move to 45 degrees
    controller.moveToPosition(1, 45.0, 0, 0, 0, true);
    
    // Get current position
    double position;
    controller.getPosition(1, position);
    std::cout << "Current position: " << position << "°" << std::endl;
    
    // Shutdown
    controller.shutdown();
    return 0;
}
```

### Three-Axis Robot Arm Example

```cpp
#include "ERobMotorController.hpp"

class ThreeAxisRobotArm {
private:
    ERobMotorController motor_controller_;
    
public:
    ThreeAxisRobotArm(const std::string& interface) 
        : motor_controller_(interface, 1000) {}
    
    bool initialize() {
        return motor_controller_.initialize() == ControllerError::SUCCESS;
    }
    
    bool moveToJointAngles(double base, double shoulder, double elbow) {
        auto result1 = motor_controller_.moveToPosition(1, base, 0, 0, 0, false);
        auto result2 = motor_controller_.moveToPosition(2, shoulder, 0, 0, 0, false);
        auto result3 = motor_controller_.moveToPosition(3, elbow, 0, 0, 0, false);
        
        return (result1 == ControllerError::SUCCESS && 
                result2 == ControllerError::SUCCESS && 
                result3 == ControllerError::SUCCESS);
    }
    
    bool enableServoMode() {
        for (uint16_t i = 1; i <= 3; ++i) {
            if (motor_controller_.enableServoMode(i) != ControllerError::SUCCESS) {
                return false;
            }
        }
        return true;
    }
    
    void setServoTargets(double base, double shoulder, double elbow) {
        std::map<uint16_t, double> targets;
        targets[1] = base;
        targets[2] = shoulder;
        targets[3] = elbow;
        motor_controller_.setMultipleServoTargets(targets);
    }
};
```

## API Reference

### ERobMotorController Class

#### Constructor
```cpp
ERobMotorController(const std::string& interface_name, uint32_t cycle_time_us = 1000)
```
- `interface_name`: EtherCAT network interface (e.g., "enp6s0", "eth0")
- `cycle_time_us`: Control cycle time in microseconds (default: 1000µs = 1ms)

#### Initialization
- `ControllerError initialize()`: Initialize controller and scan for motors
- `ControllerError shutdown()`: Shutdown controller and close connections
- `std::vector<MotorInfo> getMotorInfo() const`: Get information about discovered motors
- `size_t getMotorCount() const`: Get number of configured motors

#### Basic Control
- `ControllerError enableMotor(uint16_t motor_id)`: Enable specified motor
- `ControllerError disableMotor(uint16_t motor_id)`: Disable specified motor
- `ControllerError emergencyStop()`: Emergency stop all motors
- `ControllerError clearErrors(uint16_t motor_id)`: Clear motor errors
- `ControllerError homeMotor(uint16_t motor_id, uint32_t timeout_ms = 30000)`: Home motor
- `ControllerError setZeroPoint(uint16_t motor_id)`: Set current position as zero

#### Status and Information
- `ControllerError getMotorStatus(uint16_t motor_id, MotorStatus& status)`: Get motor status
- `ControllerError getPosition(uint16_t motor_id, double& position)`: Get position in degrees
- `ControllerError getVelocity(uint16_t motor_id, double& velocity)`: Get velocity in deg/s
- `ControllerError getTorque(uint16_t motor_id, double& torque)`: Get torque in percentage
- `ControllerError getErrorCode(uint16_t motor_id, uint16_t& error_code)`: Get error code

#### Motion Control
- `ControllerError moveToPosition(...)`: Move to absolute position
- `ControllerError moveByPosition(...)`: Move by relative position  
- `ControllerError stopMotion(uint16_t motor_id, bool quick_stop = false)`: Stop motion

#### Servo Control
- `ControllerError enableServoMode(uint16_t motor_id, const MotionParameters& params)`: Enable servo mode
- `ControllerError disableServoMode(uint16_t motor_id)`: Disable servo mode
- `ControllerError setServoTarget(uint16_t motor_id, double target_position)`: Set servo target
- `ControllerError setMultipleServoTargets(const std::map<uint16_t, double>& targets)`: Set multiple targets
- `ControllerError isTargetReached(uint16_t motor_id, bool& reached, double tolerance = -1.0)`: Check if target reached

### Data Structures

#### MotorInfo
```cpp
struct MotorInfo {
    uint16_t slave_id;           // EtherCAT slave ID
    std::string name;            // Motor name
    uint32_t vendor_id;          // Vendor ID
    uint32_t product_code;       // Product code
    uint32_t revision_number;    // Revision number
    uint32_t serial_number;      // Serial number
    bool is_configured;          // Configuration status
    bool is_operational;         // Operational status
};
```

#### MotorStatus
```cpp
struct MotorStatus {
    MotorState state;            // Current motor state
    uint16_t status_word;        // Raw status word from motor
    int32_t actual_position;     // Current position (encoder counts)
    int32_t actual_velocity;     // Current velocity (counts/sec)
    int16_t actual_torque;       // Current torque (per mille of rated torque)
    uint16_t error_code;         // Current error code
    double position_deg;         // Position in degrees
    double velocity_deg_s;       // Velocity in degrees/second
    bool is_homed;               // Homing status
    bool is_enabled;             // Motor enable status
    bool has_error;              // Error flag
    std::chrono::steady_clock::time_point timestamp; // Status timestamp
};
```

#### MotionParameters
```cpp
struct MotionParameters {
    double max_velocity;         // Maximum velocity (deg/s)
    double max_acceleration;     // Maximum acceleration (deg/s²)
    double max_deceleration;     // Maximum deceleration (deg/s²)
    double jerk_limit;           // Jerk limitation (deg/s³)
    double position_tolerance;   // Position tolerance (deg)
    double velocity_tolerance;   // Velocity tolerance (deg/s)
    uint32_t cycle_time_us;      // Control cycle time (microseconds)
};
```

### Enums

#### ControllerError
- `SUCCESS = 0`: Operation successful
- `INITIALIZATION_FAILED = 1`: Failed to initialize EtherCAT
- `NO_SLAVES_FOUND = 2`: No EtherCAT slaves found
- `CONFIGURATION_FAILED = 3`: PDO configuration failed
- `COMMUNICATION_ERROR = 4`: EtherCAT communication error
- `INVALID_MOTOR_ID = 5`: Invalid motor ID specified
- `MOTOR_NOT_OPERATIONAL = 6`: Motor not in operational state
- `TRAJECTORY_ERROR = 7`: Trajectory planning error
- `TIMEOUT_ERROR = 8`: Operation timeout
- `EMERGENCY_STOP_ACTIVE = 9`: Emergency stop is active
- `UNKNOWN_ERROR = 10`: Unknown error occurred

#### MotorState (DS402 Standard)
- `NOT_READY_TO_SWITCH_ON = 0`: Initial state
- `SWITCH_ON_DISABLED = 1`: Voltage present but motor disabled
- `READY_TO_SWITCH_ON = 2`: Ready to enable
- `SWITCHED_ON = 3`: Enabled but not operational
- `OPERATION_ENABLED = 4`: Fully operational
- `QUICK_STOP_ACTIVE = 5`: Quick stop active
- `FAULT_REACTION_ACTIVE = 6`: Fault reaction in progress
- `FAULT = 7`: Fault state
- `UNKNOWN = 8`: Unknown state

#### OperationMode (CANopen DS402)
- `PROFILE_POSITION = 1`: Profile Position Mode
- `VELOCITY = 2`: Velocity Mode
- `PROFILE_VELOCITY = 3`: Profile Velocity Mode
- `PROFILE_TORQUE = 4`: Profile Torque Mode
- `HOMING = 6`: Homing Mode
- `INTERPOLATED_POSITION = 7`: Interpolated Position Mode
- `CYCLIC_SYNC_POSITION = 8`: Cyclic Synchronous Position Mode
- `CYCLIC_SYNC_VELOCITY = 9`: Cyclic Synchronous Velocity Mode
- `CYCLIC_SYNC_TORQUE = 10`: Cyclic Synchronous Torque Mode

## System Requirements

### Hardware Requirements
- EtherCAT-compatible servo motors
- EtherCAT master interface (network adapter)
- Linux system with real-time capabilities (recommended)

### Software Requirements
- Linux operating system (Ubuntu 18.04+ recommended)
- CMake 3.9 or higher
- GCC with C++14 support
- SOEM library (included)
- Root privileges for EtherCAT access

### Network Configuration
1. Configure EtherCAT interface:
```bash
sudo ip link set enp6s0 up
sudo ethtool -s enp6s0 autoneg off speed 100 duplex full
```

2. Disable network manager for EtherCAT interface:
```bash
sudo systemctl stop NetworkManager
```

## Examples

The library includes several example programs:

### controller_test
Basic functionality test without hardware:
```bash
./controller_test [interface_name]
```

### three_axis_example
Full three-axis robot arm demonstration:
```bash
sudo ./three_axis_example [interface_name]
```

Features demonstrated:
- Motor initialization and configuration
- Homing sequence
- Point-to-point movements
- Servo mode with circular motion
- Coordinated multi-axis control

## Safety Considerations

### Important Safety Notes
- **Always run emergency stop testing** before operating with actual hardware
- **Ensure proper motor power supply** and safety circuits
- **Verify motion limits** and collision avoidance
- **Use proper cable shielding** for EtherCAT networks
- **Implement mechanical safety stops** on robot arms

### Error Handling
The library provides comprehensive error reporting:
```cpp
auto result = controller.moveToPosition(1, 45.0);
if (result != ControllerError::SUCCESS) {
    std::cerr << "Movement failed: " 
              << ERobMotorController::getErrorString(result) << std::endl;
    controller.emergencyStop();
}
```

### Emergency Stop Implementation
```cpp
// Emergency stop all motors immediately
controller.emergencyStop();

// Clear errors after resolving the issue
for (uint16_t i = 1; i <= motor_count; ++i) {
    controller.clearErrors(i);
}
```

## Troubleshooting

### Common Issues

1. **"No socket connection on interface"**
   - Run as root: `sudo ./program`
   - Check interface name: `ip link show`
   - Verify EtherCAT cables and connections

2. **"No slaves found"**
   - Check EtherCAT wiring and termination
   - Verify motor power supply
   - Confirm motor EtherCAT configuration

3. **"Communication error"**
   - Check cycle time configuration
   - Verify real-time system setup
   - Monitor EtherCAT diagnostics

4. **"Motor not operational"**
   - Check motor enable sequence
   - Verify PDO configuration
   - Clear any motor faults

### Debug Output
Enable debug output for troubleshooting:
```cpp
// The library automatically prints status information
// Monitor console output for diagnostic messages
```

### EtherCAT Diagnostics
```bash
# Check EtherCAT slaves
sudo ./slaveinfo enp6s0

# Monitor EtherCAT traffic
sudo tcpdump -i enp6s0
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make changes with proper testing
4. Submit a pull request

### Development Guidelines
- Follow existing code style
- Add comprehensive comments
- Include unit tests for new features
- Update documentation for API changes

## License

This project is licensed under the same terms as the SOEM library. See LICENSE file for details.

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review existing issues on GitHub
3. Create a new issue with detailed information
4. Include system information and error messages

## Changelog

### Version 1.0.0
- Initial release
- Full DS402 state machine implementation
- Real-time servo control with trajectory planning
- Multi-motor coordination support
- Comprehensive error handling
- Three-axis robot arm example
- Complete API documentation