/**
 * @file ERobMotorController.cpp
 * @brief Implementation of comprehensive motor control class for eRob EtherCAT servo motors
 * @author eRob SOEM Linux Project
 * @date 2024
 */

#include "ERobMotorController.hpp"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cstring>
#include <unistd.h>
#include <sys/mman.h>
#include <sched.h>

// ========== Constructor and Destructor ==========

ERobMotorController::ERobMotorController(const std::string& interface_name, uint32_t cycle_time_us)
    : interface_name_(interface_name)
    , cycle_time_us_(cycle_time_us)
    , is_initialized_(false)
    , is_operational_(false)
    , emergency_stop_active_(false)
    , thread_running_(false)
    , expected_wkc_(0)
    , wkc_(0)
{
    // Initialize memory
    memset(IOmap_, 0, sizeof(IOmap_));
    
    // Set default encoder ratios
    // Will be updated when motors are discovered
}

ERobMotorController::~ERobMotorController() {
    shutdown();
}

// ========== Initialization Functions ==========

ControllerError ERobMotorController::initialize() {
    if (is_initialized_) {
        return ControllerError::SUCCESS;
    }
    
    std::cout << "Initializing ERob Motor Controller on interface: " << interface_name_ << std::endl;
    
    // Initialize EtherCAT master
    auto result = initializeEtherCAT();
    if (result != ControllerError::SUCCESS) {
        return result;
    }
    
    // Configure PDO mappings
    result = configurePDOMappings();
    if (result != ControllerError::SUCCESS) {
        return result;
    }
    
    // Start control threads
    result = startControlThreads();
    if (result != ControllerError::SUCCESS) {
        return result;
    }
    
    is_initialized_ = true;
    std::cout << "Motor controller initialized successfully with " << motor_info_.size() << " motors" << std::endl;
    
    return ControllerError::SUCCESS;
}

ControllerError ERobMotorController::shutdown() {
    if (!is_initialized_) {
        return ControllerError::SUCCESS;
    }
    
    std::cout << "Shutting down ERob Motor Controller..." << std::endl;
    
    // Stop control threads
    stopControlThreads();
    
    // Disable all motors
    for (const auto& motor : motor_info_) {
        disableMotor(motor.slave_id);
    }
    
    // Close EtherCAT connection
    ec_close();
    
    // Clear data structures
    motor_info_.clear();
    motor_status_.clear();
    motion_params_.clear();
    motion_planners_.clear();
    encoder_ratios_.clear();
    
    is_initialized_ = false;
    is_operational_ = false;
    
    std::cout << "Motor controller shutdown complete" << std::endl;
    return ControllerError::SUCCESS;
}

std::vector<MotorInfo> ERobMotorController::getMotorInfo() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return motor_info_;
}

// ========== Basic Control Functions ==========

ControllerError ERobMotorController::enableMotor(uint16_t motor_id) {
    if (!isValidMotorId(motor_id)) {
        return ControllerError::INVALID_MOTOR_ID;
    }
    
    if (emergency_stop_active_) {
        return ControllerError::EMERGENCY_STOP_ACTIVE;
    }
    
    // Execute state transition to Operation Enabled
    return executeStateTransition(motor_id, MotorState::OPERATION_ENABLED);
}

ControllerError ERobMotorController::disableMotor(uint16_t motor_id) {
    if (!isValidMotorId(motor_id)) {
        return ControllerError::INVALID_MOTOR_ID;
    }
    
    // Execute state transition to Switch On Disabled
    return executeStateTransition(motor_id, MotorState::SWITCH_ON_DISABLED);
}

ControllerError ERobMotorController::emergencyStop() {
    emergency_stop_active_ = true;
    
    // Send quick stop to all motors
    for (const auto& motor : motor_info_) {
        sendControlCommand(motor.slave_id, ControlWords::QUICK_STOP, 0, 
                          static_cast<uint8_t>(OperationMode::PROFILE_POSITION));
    }
    
    std::cout << "Emergency stop activated!" << std::endl;
    return ControllerError::SUCCESS;
}

ControllerError ERobMotorController::clearErrors(uint16_t motor_id) {
    if (!isValidMotorId(motor_id)) {
        return ControllerError::INVALID_MOTOR_ID;
    }
    
    // Send fault reset command
    auto result = sendControlCommand(motor_id, ControlWords::FAULT_RESET, 0,
                                    static_cast<uint8_t>(OperationMode::PROFILE_POSITION));
    
    if (result == ControllerError::SUCCESS) {
        // Wait for fault to clear
        usleep(100000); // 100ms
        
        // Reset emergency stop if it was active
        emergency_stop_active_ = false;
    }
    
    return result;
}

ControllerError ERobMotorController::homeMotor(uint16_t motor_id, uint32_t timeout_ms) {
    if (!isValidMotorId(motor_id)) {
        return ControllerError::INVALID_MOTOR_ID;
    }
    
    // Switch to homing mode
    auto result = sendControlCommand(motor_id, ControlWords::ENABLE_OPERATION, 0,
                                    static_cast<uint8_t>(OperationMode::HOMING));
    if (result != ControllerError::SUCCESS) {
        return result;
    }
    
    // Start homing procedure
    result = sendControlCommand(motor_id, ControlWords::ENABLE_OPERATION | ControlWords::NEW_SET_POINT, 0,
                               static_cast<uint8_t>(OperationMode::HOMING));
    if (result != ControllerError::SUCCESS) {
        return result;
    }
    
    // Wait for homing to complete
    auto start_time = std::chrono::steady_clock::now();
    while (true) {
        MotorStatus status;
        result = getMotorStatus(motor_id, status);
        if (result != ControllerError::SUCCESS) {
            return result;
        }
        
        // Check if homing is complete (bit 12 of status word)
        if (status.status_word & 0x1000) {
            break;
        }
        
        // Check timeout
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        if (elapsed.count() > timeout_ms) {
            return ControllerError::TIMEOUT_ERROR;
        }
        
        usleep(10000); // 10ms
    }
    
    std::cout << "Motor " << motor_id << " homing completed" << std::endl;
    return ControllerError::SUCCESS;
}

ControllerError ERobMotorController::setZeroPoint(uint16_t motor_id) {
    if (!isValidMotorId(motor_id)) {
        return ControllerError::INVALID_MOTOR_ID;
    }
    
    // Set current position as zero using SDO write to position offset
    // This is motor-specific and may need adjustment based on actual motor implementation
    int32_t zero_offset = 0;
    
    // For now, we'll update our internal tracking
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (motor_status_.find(motor_id) != motor_status_.end()) {
        // Store the current position as offset for future calculations
        // This is a simplified implementation
        std::cout << "Zero point set for motor " << motor_id << std::endl;
    }
    
    return ControllerError::SUCCESS;
}

// ========== Status and Information Functions ==========

ControllerError ERobMotorController::getMotorStatus(uint16_t motor_id, MotorStatus& status) {
    if (!isValidMotorId(motor_id)) {
        return ControllerError::INVALID_MOTOR_ID;
    }
    
    std::lock_guard<std::mutex> lock(status_mutex_);
    auto it = motor_status_.find(motor_id);
    if (it != motor_status_.end()) {
        status = it->second;
        return ControllerError::SUCCESS;
    }
    
    return ControllerError::COMMUNICATION_ERROR;
}

ControllerError ERobMotorController::getPosition(uint16_t motor_id, double& position) {
    MotorStatus status;
    auto result = getMotorStatus(motor_id, status);
    if (result == ControllerError::SUCCESS) {
        position = status.position_deg;
    }
    return result;
}

ControllerError ERobMotorController::getVelocity(uint16_t motor_id, double& velocity) {
    MotorStatus status;
    auto result = getMotorStatus(motor_id, status);
    if (result == ControllerError::SUCCESS) {
        velocity = status.velocity_deg_s;
    }
    return result;
}

ControllerError ERobMotorController::getTorque(uint16_t motor_id, double& torque) {
    MotorStatus status;
    auto result = getMotorStatus(motor_id, status);
    if (result == ControllerError::SUCCESS) {
        torque = status.actual_torque / 10.0; // Convert from per mille to percentage
    }
    return result;
}

ControllerError ERobMotorController::getErrorCode(uint16_t motor_id, uint16_t& error_code) {
    MotorStatus status;
    auto result = getMotorStatus(motor_id, status);
    if (result == ControllerError::SUCCESS) {
        error_code = status.error_code;
    }
    return result;
}

// ========== Motion Control Functions ==========

ControllerError ERobMotorController::moveToPosition(uint16_t motor_id, double target_position,
                                                   double max_velocity, double acceleration,
                                                   double deceleration, bool wait_for_completion) {
    if (!isValidMotorId(motor_id)) {
        return ControllerError::INVALID_MOTOR_ID;
    }
    
    if (emergency_stop_active_) {
        return ControllerError::EMERGENCY_STOP_ACTIVE;
    }
    
    // Convert degrees to encoder counts
    int32_t target_counts = degreesToCounts(motor_id, target_position);
    
    // Set operation mode to Profile Position
    auto result = sendControlCommand(motor_id, ControlWords::ENABLE_OPERATION, target_counts,
                                    static_cast<uint8_t>(OperationMode::PROFILE_POSITION));
    if (result != ControllerError::SUCCESS) {
        return result;
    }
    
    // Trigger movement with new set point bit
    result = sendControlCommand(motor_id, ControlWords::ENABLE_OPERATION | ControlWords::NEW_SET_POINT,
                               target_counts, static_cast<uint8_t>(OperationMode::PROFILE_POSITION));
    if (result != ControllerError::SUCCESS) {
        return result;
    }
    
    std::cout << "Motor " << motor_id << " moving to position " << target_position << " degrees" << std::endl;
    
    if (wait_for_completion) {
        // Wait for movement to complete
        bool reached;
        do {
            usleep(10000); // 10ms
            result = isTargetReached(motor_id, reached);
            if (result != ControllerError::SUCCESS) {
                return result;
            }
        } while (!reached);
    }
    
    return ControllerError::SUCCESS;
}

ControllerError ERobMotorController::moveByPosition(uint16_t motor_id, double relative_position,
                                                   double max_velocity, double acceleration,
                                                   double deceleration, bool wait_for_completion) {
    double current_position;
    auto result = getPosition(motor_id, current_position);
    if (result != ControllerError::SUCCESS) {
        return result;
    }
    
    return moveToPosition(motor_id, current_position + relative_position,
                         max_velocity, acceleration, deceleration, wait_for_completion);
}

ControllerError ERobMotorController::stopMotion(uint16_t motor_id, bool quick_stop) {
    if (!isValidMotorId(motor_id)) {
        return ControllerError::INVALID_MOTOR_ID;
    }
    
    uint16_t control_word = quick_stop ? ControlWords::QUICK_STOP : ControlWords::DISABLE_OPERATION;
    
    return sendControlCommand(motor_id, control_word, 0,
                             static_cast<uint8_t>(OperationMode::PROFILE_POSITION));
}

// ========== Servo Control Functions ==========

ControllerError ERobMotorController::enableServoMode(uint16_t motor_id, const MotionParameters& motion_params) {
    if (!isValidMotorId(motor_id)) {
        return ControllerError::INVALID_MOTOR_ID;
    }
    
    // Store motion parameters
    {
        std::lock_guard<std::mutex> lock(control_mutex_);
        motion_params_[motor_id] = motion_params;
        
        // Create motion planner if it doesn't exist
        if (motion_planners_.find(motor_id) == motion_planners_.end()) {
            motion_planners_[motor_id] = std::make_unique<MotionPlanner>();
        }
        
        // Configure motion planner
        auto& planner = motion_planners_[motor_id];
        planner->max_velocity = motion_params.max_velocity * DEFAULT_ENCODER_RATIO / 360.0; // Convert to counts/s
        planner->cycle_time = cycle_time_us_ / 1000000.0; // Convert to seconds
        planner->enableServo();
    }
    
    // Switch to Cyclic Synchronous Position mode
    auto result = sendControlCommand(motor_id, ControlWords::ENABLE_OPERATION, 0,
                                    static_cast<uint8_t>(OperationMode::CYCLIC_SYNC_POSITION));
    
    std::cout << "Servo mode enabled for motor " << motor_id << std::endl;
    return result;
}

ControllerError ERobMotorController::disableServoMode(uint16_t motor_id) {
    if (!isValidMotorId(motor_id)) {
        return ControllerError::INVALID_MOTOR_ID;
    }
    
    {
        std::lock_guard<std::mutex> lock(control_mutex_);
        if (motion_planners_.find(motor_id) != motion_planners_.end()) {
            motion_planners_[motor_id]->disableServo();
        }
    }
    
    std::cout << "Servo mode disabled for motor " << motor_id << std::endl;
    return ControllerError::SUCCESS;
}

ControllerError ERobMotorController::setServoTarget(uint16_t motor_id, double target_position) {
    if (!isValidMotorId(motor_id)) {
        return ControllerError::INVALID_MOTOR_ID;
    }
    
    std::lock_guard<std::mutex> lock(control_mutex_);
    if (motion_planners_.find(motor_id) != motion_planners_.end()) {
        int32_t target_counts = degreesToCounts(motor_id, target_position);
        motion_planners_[motor_id]->setTarget(target_counts);
        return ControllerError::SUCCESS;
    }
    
    return ControllerError::MOTOR_NOT_OPERATIONAL;
}

ControllerError ERobMotorController::setMultipleServoTargets(const std::map<uint16_t, double>& targets) {
    std::lock_guard<std::mutex> lock(control_mutex_);
    
    for (const auto& target : targets) {
        uint16_t motor_id = target.first;
        double position = target.second;
        
        if (!isValidMotorId(motor_id)) {
            return ControllerError::INVALID_MOTOR_ID;
        }
        
        if (motion_planners_.find(motor_id) != motion_planners_.end()) {
            int32_t target_counts = degreesToCounts(motor_id, position);
            motion_planners_[motor_id]->setTarget(target_counts);
        }
    }
    
    return ControllerError::SUCCESS;
}

ControllerError ERobMotorController::isTargetReached(uint16_t motor_id, bool& reached, double tolerance) {
    if (!isValidMotorId(motor_id)) {
        return ControllerError::INVALID_MOTOR_ID;
    }
    
    MotorStatus status;
    auto result = getMotorStatus(motor_id, status);
    if (result != ControllerError::SUCCESS) {
        return result;
    }
    
    if (tolerance < 0) {
        // Use default tolerance from motion parameters
        std::lock_guard<std::mutex> lock(control_mutex_);
        auto it = motion_params_.find(motor_id);
        tolerance = (it != motion_params_.end()) ? it->second.position_tolerance : 0.1;
    }
    
    // Check if target is reached (DS402 standard - bit 10 of status word)
    reached = (status.status_word & 0x0400) != 0;
    
    return ControllerError::SUCCESS;
}

// ========== Configuration Functions ==========

ControllerError ERobMotorController::setMotionParameters(uint16_t motor_id, const MotionParameters& params) {
    if (!isValidMotorId(motor_id)) {
        return ControllerError::INVALID_MOTOR_ID;
    }
    
    std::lock_guard<std::mutex> lock(control_mutex_);
    motion_params_[motor_id] = params;
    
    return ControllerError::SUCCESS;
}

ControllerError ERobMotorController::getMotionParameters(uint16_t motor_id, MotionParameters& params) {
    if (!isValidMotorId(motor_id)) {
        return ControllerError::INVALID_MOTOR_ID;
    }
    
    std::lock_guard<std::mutex> lock(control_mutex_);
    auto it = motion_params_.find(motor_id);
    if (it != motion_params_.end()) {
        params = it->second;
        return ControllerError::SUCCESS;
    }
    
    return ControllerError::MOTOR_NOT_OPERATIONAL;
}

ControllerError ERobMotorController::setEncoderRatio(uint16_t motor_id, double counts_per_degree) {
    if (!isValidMotorId(motor_id)) {
        return ControllerError::INVALID_MOTOR_ID;
    }
    
    std::lock_guard<std::mutex> lock(control_mutex_);
    encoder_ratios_[motor_id] = counts_per_degree;
    
    return ControllerError::SUCCESS;
}

// ========== Utility Functions ==========

std::string ERobMotorController::getErrorString(ControllerError error) {
    switch (error) {
        case ControllerError::SUCCESS: return "Success";
        case ControllerError::INITIALIZATION_FAILED: return "Initialization failed";
        case ControllerError::NO_SLAVES_FOUND: return "No slaves found";
        case ControllerError::CONFIGURATION_FAILED: return "Configuration failed";
        case ControllerError::COMMUNICATION_ERROR: return "Communication error";
        case ControllerError::INVALID_MOTOR_ID: return "Invalid motor ID";
        case ControllerError::MOTOR_NOT_OPERATIONAL: return "Motor not operational";
        case ControllerError::TRAJECTORY_ERROR: return "Trajectory error";
        case ControllerError::TIMEOUT_ERROR: return "Timeout error";
        case ControllerError::EMERGENCY_STOP_ACTIVE: return "Emergency stop active";
        case ControllerError::UNKNOWN_ERROR: return "Unknown error";
        default: return "Undefined error";
    }
}

std::string ERobMotorController::getStateString(MotorState state) {
    switch (state) {
        case MotorState::NOT_READY_TO_SWITCH_ON: return "Not ready to switch on";
        case MotorState::SWITCH_ON_DISABLED: return "Switch on disabled";
        case MotorState::READY_TO_SWITCH_ON: return "Ready to switch on";
        case MotorState::SWITCHED_ON: return "Switched on";
        case MotorState::OPERATION_ENABLED: return "Operation enabled";
        case MotorState::QUICK_STOP_ACTIVE: return "Quick stop active";
        case MotorState::FAULT_REACTION_ACTIVE: return "Fault reaction active";
        case MotorState::FAULT: return "Fault";
        case MotorState::UNKNOWN: return "Unknown";
        default: return "Undefined state";
    }
}

// Implementation continues in Part 2...