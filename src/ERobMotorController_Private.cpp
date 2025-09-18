/**
 * @file ERobMotorController_Private.cpp  
 * @brief Private implementation methods for ERobMotorController class
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

// ========== Motion Planner Implementation ==========

int32_t MotionPlanner::planTrajectory(int32_t actual_position) {
    if (!servo_enabled) {
        return actual_position;
    }
    
    // Initialize on first start
    if (!is_moving) {
        start_position = actual_position;
        current_position = actual_position;
        smooth_target = actual_position;
        current_velocity = 0.0;
        is_moving = true;
    }
    
    // Smooth target tracking
    double target_error = target_position - smooth_target;
    if (std::abs(target_error) > 1.0) {
        smooth_target += target_error * smooth_factor;
    } else {
        smooth_target = target_position;
    }
    
    double pos_error = smooth_target - current_position;
    
    // If very close to target, use simple proportional control
    if (std::abs(pos_error) < 1.0) {
        current_position = smooth_target;
        current_velocity = 0.0;
        return static_cast<int32_t>(current_position);
    }
    
    // Distance-based velocity calculation for smooth deceleration
    double distance_to_target = std::abs(pos_error);
    double deceleration_distance = (current_velocity * current_velocity) / (2.0 * max_velocity);
    
    double desired_vel;
    if (distance_to_target > deceleration_distance) {
        // Accelerate towards max velocity
        desired_vel = std::copysign(max_velocity, pos_error);
    } else {
        // Decelerate based on remaining distance
        double vel_magnitude = std::sqrt(2.0 * max_velocity * distance_to_target);
        desired_vel = std::copysign(vel_magnitude, pos_error);
    }
    
    // Limit velocity change (acceleration)
    double vel_error = desired_vel - current_velocity;
    double max_vel_change = max_velocity * cycle_time;
    
    if (std::abs(vel_error) > max_vel_change) {
        current_velocity += std::copysign(max_vel_change, vel_error);
    } else {
        current_velocity = desired_vel;
    }
    
    // Update position based on current velocity
    current_position += current_velocity * cycle_time;
    
    return static_cast<int32_t>(current_position);
}

void MotionPlanner::setTarget(int32_t target) {
    target_position = target;
}

void MotionPlanner::enableServo() {
    servo_enabled = true;
    is_moving = false; // Reset for initialization
}

void MotionPlanner::disableServo() {
    servo_enabled = false;
    is_moving = false;
    current_velocity = 0.0;
}

// ========== ERobMotorController Private Methods ==========

ControllerError ERobMotorController::initializeEtherCAT() {
    std::cout << "Initializing EtherCAT master on interface: " << interface_name_ << std::endl;
    
    // Initialize EtherCAT master
    if (ec_init(interface_name_.c_str()) <= 0) {
        std::cerr << "Error: Could not initialize EtherCAT master!" << std::endl;
        std::cerr << "No socket connection on interface " << interface_name_ 
                  << ". Execute as root." << std::endl;
        return ControllerError::INITIALIZATION_FAILED;
    }
    
    std::cout << "EtherCAT master initialized successfully." << std::endl;
    
    // Search for EtherCAT slaves
    if (ec_config_init(FALSE) <= 0) {
        std::cerr << "Error: Cannot find EtherCAT slaves!" << std::endl;
        ec_close();
        return ControllerError::NO_SLAVES_FOUND;
    }
    
    std::cout << ec_slavecount << " slaves found and configured." << std::endl;
    
    // Collect motor information
    motor_info_.clear();
    for (int i = 1; i <= ec_slavecount; i++) {
        MotorInfo info;
        info.slave_id = i;
        info.name = std::string(ec_slave[i].name);
        info.vendor_id = ec_slave[i].eep_id;
        info.product_code = ec_slave[i].eep_id; // Simplified - would need actual product code
        info.revision_number = 0; // Would need to read from slave
        info.serial_number = 0;   // Would need to read from slave
        info.is_configured = false;
        info.is_operational = false;
        
        motor_info_.push_back(info);
        
        // Initialize encoder ratio with default value
        encoder_ratios_[i] = DEFAULT_ENCODER_RATIO;
        
        // Initialize motion parameters with defaults
        motion_params_[i] = MotionParameters();
        
        std::cout << "Found motor " << i << ": " << info.name << std::endl;
    }
    
    return ControllerError::SUCCESS;
}

ControllerError ERobMotorController::configurePDOMappings() {
    std::cout << "Configuring PDO mappings..." << std::endl;
    
    // Based on the existing demo code pattern
    for (int i = 1; i <= ec_slavecount; i++) {
        int retval = 0;
        uint16_t clear_val = 0x0000;
        uint32_t map_object;
        uint16_t map_1c12, map_1c13;
        
        // Clear RXPDO mapping (0x1600)
        retval += ec_SDOwrite(i, 0x1600, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);
        
        // Configure RXPDO mapping entries
        map_object = 0x60400010;  // Control Word, 16 bits
        retval += ec_SDOwrite(i, 0x1600, 0x01, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        map_object = 0x607A0020;  // Target Position, 32 bits
        retval += ec_SDOwrite(i, 0x1600, 0x02, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        map_object = 0x60600008;  // Mode of Operation, 8 bits
        retval += ec_SDOwrite(i, 0x1600, 0x03, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Set number of entries
        clear_val = 0x0003;
        retval += ec_SDOwrite(i, 0x1600, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);
        
        // Assign RXPDO to sync manager
        clear_val = 0x0000;
        retval += ec_SDOwrite(i, 0x1c12, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);
        
        map_1c12 = 0x1600;
        retval += ec_SDOwrite(i, 0x1c12, 0x01, FALSE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
        
        map_1c12 = 0x0001;
        retval += ec_SDOwrite(i, 0x1c12, 0x00, FALSE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
        
        // Clear TXPDO mapping (0x1A00)
        clear_val = 0x0000;
        retval += ec_SDOwrite(i, 0x1A00, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);
        
        // Configure TXPDO mapping entries
        map_object = 0x60410010;  // Status Word, 16 bits
        retval += ec_SDOwrite(i, 0x1A00, 0x01, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        map_object = 0x60640020;  // Actual Position, 32 bits
        retval += ec_SDOwrite(i, 0x1A00, 0x02, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        map_object = 0x606C0020;  // Actual Velocity, 32 bits
        retval += ec_SDOwrite(i, 0x1A00, 0x03, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        map_object = 0x60770010;  // Actual Torque, 16 bits
        retval += ec_SDOwrite(i, 0x1A00, 0x04, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Set number of entries
        clear_val = 0x0004;
        retval += ec_SDOwrite(i, 0x1A00, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);
        
        // Assign TXPDO to sync manager
        clear_val = 0x0000;
        retval += ec_SDOwrite(i, 0x1C13, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);
        
        map_1c13 = 0x1A00;
        retval += ec_SDOwrite(i, 0x1C13, 0x01, FALSE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);
        
        map_1c13 = 0x0001;
        retval += ec_SDOwrite(i, 0x1C13, 0x00, FALSE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);
        
        if (retval < 0) {
            std::cerr << "PDO mapping failed for slave " << i << std::endl;
            return ControllerError::CONFIGURATION_FAILED;
        }
        
        std::cout << "PDO mapping configured for slave " << i << std::endl;
    }
    
    // Map the configured PDOs to the IOmap
    ec_config_map(&IOmap_);
    
    // Configure distributed clock synchronization
    for (int i = 1; i <= ec_slavecount; i++) {
        ec_dcsync0(i, TRUE, cycle_time_us_ * 1000, 0);  // Convert to nanoseconds
    }
    
    // Change slaves to operational state
    ec_writestate(0);
    
    // Wait for all slaves to reach operational state
    int wait_timeout = 50;  // 5 seconds
    do {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_readstate();
        
        bool all_operational = true;
        for (int i = 1; i <= ec_slavecount; i++) {
            if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                all_operational = false;
                break;
            }
        }
        
        if (all_operational) {
            break;
        }
        
        usleep(100000); // 100ms
        wait_timeout--;
    } while (wait_timeout > 0);
    
    if (wait_timeout == 0) {
        std::cerr << "Not all slaves reached operational state" << std::endl;
        return ControllerError::CONFIGURATION_FAILED;
    }
    
    expected_wkc_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    std::cout << "Expected working counter: " << expected_wkc_ << std::endl;
    std::cout << "All slaves operational, configuring servo motors..." << std::endl;
    
    // Configure servo motor parameters
    for (int i = 1; i <= ec_slavecount; i++) {
        uint8_t operation_mode = static_cast<uint8_t>(OperationMode::PROFILE_POSITION);
        uint16_t control_word = ControlWords::SHUTDOWN;
        uint32_t profile_velocity = 50000;
        uint32_t profile_acceleration = 150000;
        uint32_t profile_deceleration = 150000;
        
        ec_SDOwrite(i, 0x6040, 0x00, FALSE, sizeof(control_word), &control_word, EC_TIMEOUTSAFE);
        ec_SDOwrite(i, 0x6060, 0x00, FALSE, sizeof(operation_mode), &operation_mode, EC_TIMEOUTSAFE);
        ec_SDOwrite(i, 0x6081, 0x00, FALSE, sizeof(profile_velocity), &profile_velocity, EC_TIMEOUTSAFE);
        ec_SDOwrite(i, 0x6083, 0x00, FALSE, sizeof(profile_acceleration), &profile_acceleration, EC_TIMEOUTSAFE);
        ec_SDOwrite(i, 0x6084, 0x00, FALSE, sizeof(profile_deceleration), &profile_deceleration, EC_TIMEOUTSAFE);
        
        // Mark motor as configured
        motor_info_[i-1].is_configured = true;
        motor_info_[i-1].is_operational = true;
    }
    
    std::cout << "PDO mapping and servo configuration complete." << std::endl;
    return ControllerError::SUCCESS;
}

ControllerError ERobMotorController::startControlThreads() {
    if (thread_running_) {
        return ControllerError::SUCCESS;
    }
    
    thread_running_ = true;
    
    // Start real-time control thread
    control_thread_ = std::make_unique<std::thread>(&ERobMotorController::controlThreadFunction, this);
    
    // Start monitoring thread
    monitor_thread_ = std::make_unique<std::thread>(&ERobMotorController::monitorThreadFunction, this);
    
    std::cout << "Control threads started." << std::endl;
    return ControllerError::SUCCESS;
}

void ERobMotorController::stopControlThreads() {
    if (!thread_running_) {
        return;
    }
    
    thread_running_ = false;
    
    if (control_thread_ && control_thread_->joinable()) {
        control_thread_->join();
    }
    
    if (monitor_thread_ && monitor_thread_->joinable()) {
        monitor_thread_->join();
    }
    
    control_thread_.reset();
    monitor_thread_.reset();
    
    std::cout << "Control threads stopped." << std::endl;
}

void ERobMotorController::controlThreadFunction() {
    std::cout << "Real-time control thread started." << std::endl;
    
    // Set real-time priority
    struct sched_param param;
    param.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
        std::cerr << "Warning: Could not set real-time priority" << std::endl;
    }
    
    auto next_cycle = std::chrono::steady_clock::now();
    auto cycle_duration = std::chrono::microseconds(cycle_time_us_);
    
    while (thread_running_) {
        // Send process data
        ec_send_processdata();
        
        // Process servo control for each motor
        {
            std::lock_guard<std::mutex> lock(control_mutex_);
            
            for (auto& planner_pair : motion_planners_) {
                uint16_t motor_id = planner_pair.first;
                auto& planner = planner_pair.second;
                
                if (planner->servo_enabled) {
                    // Get actual position from received data
                    txpdo_t* txpdo = reinterpret_cast<txpdo_t*>(ec_slave[motor_id].inputs);
                    
                    // Plan trajectory
                    int32_t planned_position = planner->planTrajectory(txpdo->actual_position);
                    
                    // Send position command
                    rxpdo_t* rxpdo = reinterpret_cast<rxpdo_t*>(ec_slave[motor_id].outputs);
                    rxpdo->controlword = ControlWords::ENABLE_OPERATION;
                    rxpdo->target_position = planned_position;
                    rxpdo->mode_of_operation = static_cast<uint8_t>(OperationMode::CYCLIC_SYNC_POSITION);
                }
            }
        }
        
        // Receive process data
        wkc_ = ec_receive_processdata(EC_TIMEOUTRET);
        
        // Check working counter
        if (wkc_ < expected_wkc_) {
            // Communication error handling
            static int error_count = 0;
            error_count++;
            if (error_count > 10) {
                std::cerr << "Communication error: WKC=" << wkc_ 
                         << ", expected=" << expected_wkc_ << std::endl;
                error_count = 0;
            }
        }
        
        // Wait for next cycle
        next_cycle += cycle_duration;
        std::this_thread::sleep_until(next_cycle);
    }
    
    std::cout << "Real-time control thread stopped." << std::endl;
}

void ERobMotorController::monitorThreadFunction() {
    std::cout << "Monitor thread started." << std::endl;
    
    while (thread_running_) {
        // Update motor status for all motors
        for (const auto& motor : motor_info_) {
            updateMotorStatus(motor.slave_id);
        }
        
        // Check operational status
        is_operational_ = (wkc_ >= expected_wkc_);
        
        // Sleep for monitor cycle
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    std::cout << "Monitor thread stopped." << std::endl;
}

void ERobMotorController::updateMotorStatus(uint16_t motor_id) {
    if (motor_id > ec_slavecount) {
        return;
    }
    
    // Get data from EtherCAT
    txpdo_t* txpdo = reinterpret_cast<txpdo_t*>(ec_slave[motor_id].inputs);
    
    MotorStatus status;
    status.status_word = txpdo->statusword;
    status.actual_position = txpdo->actual_position;
    status.actual_velocity = txpdo->actual_velocity;
    status.actual_torque = txpdo->actual_torque;
    status.error_code = 0; // Would need to read from specific error register
    
    // Convert to engineering units
    status.position_deg = countsTodegrees(motor_id, status.actual_position);
    status.velocity_deg_s = countsTodegrees(motor_id, status.actual_velocity);
    
    // Parse motor state
    status.state = parseMotorState(status.status_word);
    
    // Set flags
    status.is_enabled = (status.state == MotorState::OPERATION_ENABLED);
    status.has_error = (status.state == MotorState::FAULT || status.state == MotorState::FAULT_REACTION_ACTIVE);
    status.is_homed = (status.status_word & 0x1000) != 0; // Homing complete bit
    
    // Update timestamp
    status.timestamp = std::chrono::steady_clock::now();
    
    // Store status
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        motor_status_[motor_id] = status;
    }
}

ControllerError ERobMotorController::sendControlCommand(uint16_t motor_id, uint16_t control_word,
                                                       int32_t target_position, uint8_t operation_mode) {
    if (motor_id > ec_slavecount) {
        return ControllerError::INVALID_MOTOR_ID;
    }
    
    // Get RXPDO data pointer
    rxpdo_t* rxpdo = reinterpret_cast<rxpdo_t*>(ec_slave[motor_id].outputs);
    
    // Set control data
    rxpdo->controlword = control_word;
    rxpdo->target_position = target_position;
    rxpdo->mode_of_operation = operation_mode;
    
    return ControllerError::SUCCESS;
}

bool ERobMotorController::isValidMotorId(uint16_t motor_id) const {
    return (motor_id >= 1 && motor_id <= ec_slavecount);
}

double ERobMotorController::countsTodegrees(uint16_t motor_id, int32_t counts) const {
    auto it = encoder_ratios_.find(motor_id);
    double ratio = (it != encoder_ratios_.end()) ? it->second : DEFAULT_ENCODER_RATIO;
    return counts / ratio;
}

int32_t ERobMotorController::degreesToCounts(uint16_t motor_id, double degrees) const {
    auto it = encoder_ratios_.find(motor_id);
    double ratio = (it != encoder_ratios_.end()) ? it->second : DEFAULT_ENCODER_RATIO;
    return static_cast<int32_t>(degrees * ratio);
}

MotorState ERobMotorController::parseMotorState(uint16_t status_word) const {
    // DS402 state machine decoding
    uint16_t state_bits = status_word & 0x006F; // Mask relevant bits
    
    switch (state_bits) {
        case 0x0000: return MotorState::NOT_READY_TO_SWITCH_ON;
        case 0x0040: return MotorState::SWITCH_ON_DISABLED;
        case 0x0021: return MotorState::READY_TO_SWITCH_ON;
        case 0x0023: return MotorState::SWITCHED_ON;
        case 0x0027: return MotorState::OPERATION_ENABLED;
        case 0x0007: return MotorState::QUICK_STOP_ACTIVE;
        case 0x000F: return MotorState::FAULT_REACTION_ACTIVE;
        case 0x0008: return MotorState::FAULT;
        default: return MotorState::UNKNOWN;
    }
}

ControllerError ERobMotorController::waitForState(uint16_t motor_id, MotorState target_state, uint32_t timeout_ms) {
    auto start_time = std::chrono::steady_clock::now();
    
    while (true) {
        MotorStatus status;
        auto result = getMotorStatus(motor_id, status);
        if (result != ControllerError::SUCCESS) {
            return result;
        }
        
        if (status.state == target_state) {
            return ControllerError::SUCCESS;
        }
        
        // Check timeout
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        if (elapsed.count() > timeout_ms) {
            return ControllerError::TIMEOUT_ERROR;
        }
        
        usleep(1000); // 1ms
    }
}

ControllerError ERobMotorController::executeStateTransition(uint16_t motor_id, MotorState target_state) {
    MotorStatus current_status;
    auto result = getMotorStatus(motor_id, current_status);
    if (result != ControllerError::SUCCESS) {
        return result;
    }
    
    MotorState current_state = current_status.state;
    
    // Handle fault states first
    if (current_state == MotorState::FAULT || current_state == MotorState::FAULT_REACTION_ACTIVE) {
        result = sendControlCommand(motor_id, ControlWords::FAULT_RESET, 0,
                                   static_cast<uint8_t>(OperationMode::PROFILE_POSITION));
        if (result != ControllerError::SUCCESS) {
            return result;
        }
        
        result = waitForState(motor_id, MotorState::SWITCH_ON_DISABLED, 1000);
        if (result != ControllerError::SUCCESS) {
            return result;
        }
        
        current_state = MotorState::SWITCH_ON_DISABLED;
    }
    
    // State transition sequence
    switch (target_state) {
        case MotorState::OPERATION_ENABLED:
            // Path: Any -> Switch On Disabled -> Ready to Switch On -> Switched On -> Operation Enabled
            if (current_state != MotorState::SWITCH_ON_DISABLED) {
                result = sendControlCommand(motor_id, ControlWords::DISABLE_VOLTAGE, 0,
                                           static_cast<uint8_t>(OperationMode::PROFILE_POSITION));
                if (result != ControllerError::SUCCESS) return result;
                result = waitForState(motor_id, MotorState::SWITCH_ON_DISABLED, 1000);
                if (result != ControllerError::SUCCESS) return result;
            }
            
            // Shutdown -> Ready to Switch On
            result = sendControlCommand(motor_id, ControlWords::SHUTDOWN, 0,
                                       static_cast<uint8_t>(OperationMode::PROFILE_POSITION));
            if (result != ControllerError::SUCCESS) return result;
            result = waitForState(motor_id, MotorState::READY_TO_SWITCH_ON, 1000);
            if (result != ControllerError::SUCCESS) return result;
            
            // Switch On -> Switched On
            result = sendControlCommand(motor_id, ControlWords::SWITCH_ON, 0,
                                       static_cast<uint8_t>(OperationMode::PROFILE_POSITION));
            if (result != ControllerError::SUCCESS) return result;
            result = waitForState(motor_id, MotorState::SWITCHED_ON, 1000);
            if (result != ControllerError::SUCCESS) return result;
            
            // Enable Operation -> Operation Enabled
            result = sendControlCommand(motor_id, ControlWords::ENABLE_OPERATION, 0,
                                       static_cast<uint8_t>(OperationMode::PROFILE_POSITION));
            if (result != ControllerError::SUCCESS) return result;
            result = waitForState(motor_id, MotorState::OPERATION_ENABLED, 1000);
            break;
            
        case MotorState::SWITCH_ON_DISABLED:
            result = sendControlCommand(motor_id, ControlWords::DISABLE_VOLTAGE, 0,
                                       static_cast<uint8_t>(OperationMode::PROFILE_POSITION));
            if (result != ControllerError::SUCCESS) return result;
            result = waitForState(motor_id, MotorState::SWITCH_ON_DISABLED, 1000);
            break;
            
        default:
            return ControllerError::UNKNOWN_ERROR;
    }
    
    return result;
}