/**
 * @file ERobMotorController.hpp
 * @brief Comprehensive motor control class for eRob EtherCAT servo motors
 * @author eRob SOEM Linux Project
 * @date 2024
 * 
 * This class provides a complete motor control interface for EtherCAT servo motors,
 * including initialization, position control, servo control, and basic motor operations.
 * Designed for robotic arm applications with real-time trajectory planning.
 */

#ifndef EROB_MOTOR_CONTROLLER_HPP
#define EROB_MOTOR_CONTROLLER_HPP

#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <cmath>
#include "ethercat.h"

// Forward declarations
struct MotorInfo;
struct MotorStatus;

// PDO structures based on existing demo code
typedef struct __attribute__((__packed__)) {
    uint16_t controlword;      // 0x6040:0, 16 bits
    int32_t target_position;   // 0x607A:0, 32 bits
    uint8_t mode_of_operation; // 0x6060:0, 8 bits
    uint8_t padding;           // 8 bits padding for alignment
} rxpdo_t;

typedef struct __attribute__((__packed__)) {
    uint16_t statusword;      // 0x6041:0, 16 bits
    int32_t actual_position;  // 0x6064:0, 32 bits
    int32_t actual_velocity;  // 0x606C:0, 32 bits
    int16_t actual_torque;    // 0x6077:0, 16 bits
} txpdo_t;

// Control word definitions (DS402 standard)
namespace ControlWords {
    const uint16_t SHUTDOWN = 0x0006;
    const uint16_t SWITCH_ON = 0x0007;
    const uint16_t DISABLE_VOLTAGE = 0x0000;
    const uint16_t QUICK_STOP = 0x0002;
    const uint16_t DISABLE_OPERATION = 0x0007;
    const uint16_t ENABLE_OPERATION = 0x000F;
    const uint16_t FAULT_RESET = 0x0080;
    const uint16_t NEW_SET_POINT = 0x0010;
}

// Default encoder ratio (counts per degree) - can be configured per motor
const double DEFAULT_ENCODER_RATIO = 1.0 / 0.000686645; // From demo code

// Motion planner implementation (from existing demo code)
struct MotionPlanner {
    int32_t start_position;
    int32_t target_position;
    int32_t smooth_target;
    int32_t current_position;
    double current_velocity;
    double start_time;
    double total_time;
    double current_time;
    bool is_moving;
    bool servo_enabled;
    
    // Motion parameters
    double max_velocity;
    double cycle_time;
    double smooth_factor;
    
    MotionPlanner() : start_position(0), target_position(0), smooth_target(0),
                      current_position(0), current_velocity(0.0),
                      start_time(0.0), total_time(0.0), current_time(0.0),
                      is_moving(false), servo_enabled(false),
                      max_velocity(50000.0), cycle_time(0.001), smooth_factor(0.002) {}
                      
    int32_t planTrajectory(int32_t actual_position);
    void setTarget(int32_t target);
    void enableServo();
    void disableServo();
};

/**
 * @brief Motor operation modes based on CANopen DS402 standard
 */
enum class OperationMode : uint8_t {
    PROFILE_POSITION = 1,      // PP: Profile Position Mode
    VELOCITY = 2,              // VL: Velocity Mode  
    PROFILE_VELOCITY = 3,      // PV: Profile Velocity Mode
    PROFILE_TORQUE = 4,        // TQ: Torque Profile Mode
    HOMING = 6,                // HM: Homing Mode
    INTERPOLATED_POSITION = 7, // IP: Interpolated Position Mode
    CYCLIC_SYNC_POSITION = 8,  // CSP: Cyclic Synchronous Position Mode
    CYCLIC_SYNC_VELOCITY = 9,  // CSV: Cyclic Synchronous Velocity Mode
    CYCLIC_SYNC_TORQUE = 10    // CST: Cyclic Synchronous Torque Mode
};

/**
 * @brief Motor state enumeration based on DS402 state machine
 */
enum class MotorState {
    NOT_READY_TO_SWITCH_ON = 0,
    SWITCH_ON_DISABLED = 1,
    READY_TO_SWITCH_ON = 2,
    SWITCHED_ON = 3,
    OPERATION_ENABLED = 4,
    QUICK_STOP_ACTIVE = 5,
    FAULT_REACTION_ACTIVE = 6,
    FAULT = 7,
    UNKNOWN = 8
};

/**
 * @brief Error codes for motor controller operations
 */
enum class ControllerError {
    SUCCESS = 0,
    INITIALIZATION_FAILED = 1,
    NO_SLAVES_FOUND = 2,
    CONFIGURATION_FAILED = 3,
    COMMUNICATION_ERROR = 4,
    INVALID_MOTOR_ID = 5,
    MOTOR_NOT_OPERATIONAL = 6,
    TRAJECTORY_ERROR = 7,
    TIMEOUT_ERROR = 8,
    EMERGENCY_STOP_ACTIVE = 9,
    UNKNOWN_ERROR = 10
};

/**
 * @brief Structure containing motor information
 */
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

/**
 * @brief Structure containing real-time motor status
 */
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

/**
 * @brief Motion planning parameters for trajectory generation
 */
struct MotionParameters {
    double max_velocity;         // Maximum velocity (deg/s)
    double max_acceleration;     // Maximum acceleration (deg/s²)
    double max_deceleration;     // Maximum deceleration (deg/s²)
    double jerk_limit;           // Jerk limitation (deg/s³)
    double position_tolerance;   // Position tolerance (deg)
    double velocity_tolerance;   // Velocity tolerance (deg/s)
    uint32_t cycle_time_us;      // Control cycle time (microseconds)
    
    MotionParameters() : 
        max_velocity(360.0), max_acceleration(1800.0), max_deceleration(1800.0),
        jerk_limit(18000.0), position_tolerance(0.1), velocity_tolerance(1.0),
        cycle_time_us(1000) {}
};

/**
 * @brief Comprehensive motor controller class for EtherCAT servo motors
 * 
 * This class provides high-level control interface for EtherCAT servo motors
 * with features including:
 * - Automatic motor discovery and configuration
 * - Position and velocity control
 * - Real-time servo control with trajectory planning
 * - Error handling and safety functions
 * - Multi-motor coordination support
 */
class ERobMotorController {
public:
    /**
     * @brief Constructor
     * @param interface_name EtherCAT network interface name (e.g., "eth0", "enp6s0")
     * @param cycle_time_us Control cycle time in microseconds (default: 1000us = 1ms)
     */
    explicit ERobMotorController(const std::string& interface_name, uint32_t cycle_time_us = 1000);
    
    /**
     * @brief Destructor - ensures proper cleanup
     */
    ~ERobMotorController();

    // ========== Initialization Functions ==========
    
    /**
     * @brief Initialize the motor controller and scan for motors
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError initialize();
    
    /**
     * @brief Shutdown the controller and close all connections
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError shutdown();
    
    /**
     * @brief Get information about discovered motors
     * @return Vector of motor information structures
     */
    std::vector<MotorInfo> getMotorInfo() const;
    
    /**
     * @brief Get number of configured motors
     * @return Number of motors
     */
    size_t getMotorCount() const { return motor_info_.size(); }

    // ========== Basic Control Functions ==========
    
    /**
     * @brief Enable specified motor
     * @param motor_id Motor ID (1-based indexing)
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError enableMotor(uint16_t motor_id);
    
    /**
     * @brief Disable specified motor
     * @param motor_id Motor ID (1-based indexing)
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError disableMotor(uint16_t motor_id);
    
    /**
     * @brief Emergency stop all motors
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError emergencyStop();
    
    /**
     * @brief Clear motor errors
     * @param motor_id Motor ID (1-based indexing)
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError clearErrors(uint16_t motor_id);
    
    /**
     * @brief Home specified motor (find reference position)
     * @param motor_id Motor ID (1-based indexing)
     * @param timeout_ms Timeout in milliseconds
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError homeMotor(uint16_t motor_id, uint32_t timeout_ms = 30000);
    
    /**
     * @brief Set current position as zero point
     * @param motor_id Motor ID (1-based indexing)
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError setZeroPoint(uint16_t motor_id);

    // ========== Status and Information Functions ==========
    
    /**
     * @brief Get current motor status
     * @param motor_id Motor ID (1-based indexing)
     * @param status Reference to status structure to fill
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError getMotorStatus(uint16_t motor_id, MotorStatus& status);
    
    /**
     * @brief Get motor position in degrees
     * @param motor_id Motor ID (1-based indexing)
     * @param position Reference to store position value
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError getPosition(uint16_t motor_id, double& position);
    
    /**
     * @brief Get motor velocity in degrees per second
     * @param motor_id Motor ID (1-based indexing)
     * @param velocity Reference to store velocity value
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError getVelocity(uint16_t motor_id, double& velocity);
    
    /**
     * @brief Get motor torque in percentage of rated torque
     * @param motor_id Motor ID (1-based indexing)
     * @param torque Reference to store torque value
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError getTorque(uint16_t motor_id, double& torque);
    
    /**
     * @brief Read motor error code
     * @param motor_id Motor ID (1-based indexing)
     * @param error_code Reference to store error code
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError getErrorCode(uint16_t motor_id, uint16_t& error_code);

    // ========== Motion Control Functions ==========
    
    /**
     * @brief Move motor to absolute position with specified parameters
     * @param motor_id Motor ID (1-based indexing)
     * @param target_position Target position in degrees
     * @param max_velocity Maximum velocity in degrees/second
     * @param acceleration Acceleration in degrees/second²
     * @param deceleration Deceleration in degrees/second²
     * @param wait_for_completion Wait for movement completion
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError moveToPosition(uint16_t motor_id, double target_position, 
                                   double max_velocity = 0, double acceleration = 0, 
                                   double deceleration = 0, bool wait_for_completion = false);
    
    /**
     * @brief Move motor by relative position
     * @param motor_id Motor ID (1-based indexing)
     * @param relative_position Relative position in degrees
     * @param max_velocity Maximum velocity in degrees/second
     * @param acceleration Acceleration in degrees/second²
     * @param deceleration Deceleration in degrees/second²
     * @param wait_for_completion Wait for movement completion
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError moveByPosition(uint16_t motor_id, double relative_position,
                                   double max_velocity = 0, double acceleration = 0,
                                   double deceleration = 0, bool wait_for_completion = false);
    
    /**
     * @brief Stop motor motion
     * @param motor_id Motor ID (1-based indexing)
     * @param quick_stop Use quick stop (immediate) or controlled deceleration
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError stopMotion(uint16_t motor_id, bool quick_stop = false);

    // ========== Servo Control Functions ==========
    
    /**
     * @brief Enable servo control mode for specified motor
     * @param motor_id Motor ID (1-based indexing)
     * @param motion_params Motion parameters for trajectory planning
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError enableServoMode(uint16_t motor_id, const MotionParameters& motion_params = MotionParameters());
    
    /**
     * @brief Disable servo control mode
     * @param motor_id Motor ID (1-based indexing)
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError disableServoMode(uint16_t motor_id);
    
    /**
     * @brief Set target position for servo control
     * @param motor_id Motor ID (1-based indexing)
     * @param target_position Target position in degrees
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError setServoTarget(uint16_t motor_id, double target_position);
    
    /**
     * @brief Set multiple motor targets simultaneously (for coordinated motion)
     * @param targets Map of motor_id to target_position
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError setMultipleServoTargets(const std::map<uint16_t, double>& targets);
    
    /**
     * @brief Check if motor has reached target position
     * @param motor_id Motor ID (1-based indexing)
     * @param reached Reference to store result
     * @param tolerance Position tolerance in degrees (optional)
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError isTargetReached(uint16_t motor_id, bool& reached, double tolerance = -1.0);

    // ========== Configuration Functions ==========
    
    /**
     * @brief Set motion parameters for specified motor
     * @param motor_id Motor ID (1-based indexing)
     * @param params Motion parameters
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError setMotionParameters(uint16_t motor_id, const MotionParameters& params);
    
    /**
     * @brief Get current motion parameters
     * @param motor_id Motor ID (1-based indexing)
     * @param params Reference to store parameters
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError getMotionParameters(uint16_t motor_id, MotionParameters& params);
    
    /**
     * @brief Set encoder counts to degree conversion factor
     * @param motor_id Motor ID (1-based indexing)
     * @param counts_per_degree Encoder counts per degree
     * @return ControllerError::SUCCESS on success, error code otherwise
     */
    ControllerError setEncoderRatio(uint16_t motor_id, double counts_per_degree);

    // ========== Utility Functions ==========
    
    /**
     * @brief Check if controller is initialized and operational
     * @return true if operational, false otherwise
     */
    bool isOperational() const { return is_initialized_ && is_operational_; }
    
    /**
     * @brief Get string description of error code
     * @param error Error code
     * @return Error description string
     */
    static std::string getErrorString(ControllerError error);
    
    /**
     * @brief Get string description of motor state
     * @param state Motor state
     * @return State description string
     */
    static std::string getStateString(MotorState state);

private:
    // ========== Private Data Members ==========
    
    std::string interface_name_;                     // EtherCAT interface name
    uint32_t cycle_time_us_;                        // Control cycle time
    std::atomic<bool> is_initialized_;              // Initialization flag
    std::atomic<bool> is_operational_;              // Operational flag
    std::atomic<bool> emergency_stop_active_;      // Emergency stop flag
    
    std::vector<MotorInfo> motor_info_;             // Motor information
    std::map<uint16_t, MotorStatus> motor_status_;  // Motor status cache
    std::map<uint16_t, MotionParameters> motion_params_; // Motion parameters per motor
    std::map<uint16_t, std::unique_ptr<MotionPlanner>> motion_planners_; // Trajectory planners
    std::map<uint16_t, double> encoder_ratios_;     // Encoder counts per degree
    
    // Thread management
    std::unique_ptr<std::thread> control_thread_;   // Real-time control thread
    std::unique_ptr<std::thread> monitor_thread_;   // Status monitoring thread
    std::atomic<bool> thread_running_;              // Thread control flag
    
    // Synchronization
    mutable std::mutex status_mutex_;               // Status data protection
    mutable std::mutex control_mutex_;              // Control command protection
    std::condition_variable position_reached_cv_;   // Position reached notification
    
    // EtherCAT specific
    char IOmap_[4096];                              // EtherCAT I/O mapping
    int expected_wkc_;                              // Expected work counter
    volatile int wkc_;                              // Work counter
    
    // ========== Private Methods ==========
    
    /**
     * @brief Initialize EtherCAT master and discover slaves
     */
    ControllerError initializeEtherCAT();
    
    /**
     * @brief Configure PDO mappings for all motors
     */
    ControllerError configurePDOMappings();
    
    /**
     * @brief Start real-time control threads
     */
    ControllerError startControlThreads();
    
    /**
     * @brief Stop control threads
     */
    void stopControlThreads();
    
    /**
     * @brief Real-time control thread function
     */
    void controlThreadFunction();
    
    /**
     * @brief Status monitoring thread function
     */
    void monitorThreadFunction();
    
    /**
     * @brief Update motor status from EtherCAT data
     */
    void updateMotorStatus(uint16_t motor_id);
    
    /**
     * @brief Send control commands to motor
     */
    ControllerError sendControlCommand(uint16_t motor_id, uint16_t control_word, 
                                       int32_t target_position, uint8_t operation_mode);
    
    /**
     * @brief Validate motor ID
     */
    bool isValidMotorId(uint16_t motor_id) const;
    
    /**
     * @brief Convert encoder counts to degrees
     */
    double countsTodegrees(uint16_t motor_id, int32_t counts) const;
    
    /**
     * @brief Convert degrees to encoder counts
     */
    int32_t degreesToCounts(uint16_t motor_id, double degrees) const;
    
    /**
     * @brief Parse motor state from status word
     */
    MotorState parseMotorState(uint16_t status_word) const;
    
    /**
     * @brief Wait for motor to reach specific state
     */
    ControllerError waitForState(uint16_t motor_id, MotorState target_state, uint32_t timeout_ms);
    
    /**
     * @brief Execute state transition sequence
     */
    ControllerError executeStateTransition(uint16_t motor_id, MotorState target_state);
};

#endif // EROB_MOTOR_CONTROLLER_HPP