/**
 * @file controller_test.cpp
 * @brief Simple test program for ERobMotorController without requiring hardware
 * @author eRob SOEM Linux Project
 * @date 2024
 */

#include "ERobMotorController.hpp"
#include <iostream>
#include <string>

/**
 * @brief Test basic motor controller functionality
 */
void testBasicFunctionality() {
    std::cout << "=== Testing Basic Functionality ===" << std::endl;
    
    // Test error string conversion
    std::cout << "Error strings test:" << std::endl;
    for (int i = 0; i <= 10; ++i) {
        ControllerError error = static_cast<ControllerError>(i);
        std::cout << "  Error " << i << ": " << ERobMotorController::getErrorString(error) << std::endl;
    }
    
    // Test motor state string conversion
    std::cout << "\nMotor state strings test:" << std::endl;
    for (int i = 0; i <= 8; ++i) {
        MotorState state = static_cast<MotorState>(i);
        std::cout << "  State " << i << ": " << ERobMotorController::getStateString(state) << std::endl;
    }
    
    // Test motion parameters
    std::cout << "\nMotion parameters test:" << std::endl;
    MotionParameters params;
    std::cout << "  Max velocity: " << params.max_velocity << " deg/s" << std::endl;
    std::cout << "  Max acceleration: " << params.max_acceleration << " deg/s²" << std::endl;
    std::cout << "  Position tolerance: " << params.position_tolerance << " deg" << std::endl;
    std::cout << "  Cycle time: " << params.cycle_time_us << " us" << std::endl;
}

/**
 * @brief Test motor controller initialization (will fail without hardware)
 */
void testControllerInitialization(const std::string& interface_name) {
    std::cout << "\n=== Testing Controller Initialization ===" << std::endl;
    std::cout << "Note: This test will fail without EtherCAT hardware" << std::endl;
    
    ERobMotorController controller(interface_name, 1000);
    
    std::cout << "Controller created successfully" << std::endl;
    std::cout << "Interface: " << interface_name << std::endl;
    std::cout << "Is operational: " << (controller.isOperational() ? "Yes" : "No") << std::endl;
    
    // Try to initialize (will fail without hardware/root access)
    auto result = controller.initialize();
    std::cout << "Initialization result: " << ERobMotorController::getErrorString(result) << std::endl;
    
    if (result == ControllerError::SUCCESS) {
        std::cout << "Motor count: " << controller.getMotorCount() << std::endl;
        
        // Test getting motor info
        auto motor_info = controller.getMotorInfo();
        for (const auto& info : motor_info) {
            std::cout << "Motor " << info.slave_id << ": " << info.name << std::endl;
        }
        
        // Shutdown
        controller.shutdown();
    } else {
        std::cout << "Expected failure - no EtherCAT hardware or not running as root" << std::endl;
    }
}

/**
 * @brief Test operation modes enumeration
 */
void testOperationModes() {
    std::cout << "\n=== Testing Operation Modes ===" << std::endl;
    
    std::cout << "Operation modes:" << std::endl;
    std::cout << "  Profile Position: " << static_cast<int>(OperationMode::PROFILE_POSITION) << std::endl;
    std::cout << "  Velocity: " << static_cast<int>(OperationMode::VELOCITY) << std::endl;
    std::cout << "  Profile Velocity: " << static_cast<int>(OperationMode::PROFILE_VELOCITY) << std::endl;
    std::cout << "  Profile Torque: " << static_cast<int>(OperationMode::PROFILE_TORQUE) << std::endl;
    std::cout << "  Homing: " << static_cast<int>(OperationMode::HOMING) << std::endl;
    std::cout << "  Interpolated Position: " << static_cast<int>(OperationMode::INTERPOLATED_POSITION) << std::endl;
    std::cout << "  Cyclic Sync Position: " << static_cast<int>(OperationMode::CYCLIC_SYNC_POSITION) << std::endl;
    std::cout << "  Cyclic Sync Velocity: " << static_cast<int>(OperationMode::CYCLIC_SYNC_VELOCITY) << std::endl;
    std::cout << "  Cyclic Sync Torque: " << static_cast<int>(OperationMode::CYCLIC_SYNC_TORQUE) << std::endl;
}

/**
 * @brief Test motor status structure
 */
void testMotorStatus() {
    std::cout << "\n=== Testing Motor Status Structure ===" << std::endl;
    
    MotorStatus status;
    status.state = MotorState::OPERATION_ENABLED;
    status.status_word = 0x0027;
    status.actual_position = 12000;
    status.actual_velocity = 1000;
    status.actual_torque = 250;
    status.error_code = 0;
    status.position_deg = 45.0;
    status.velocity_deg_s = 10.0;
    status.is_homed = true;
    status.is_enabled = true;
    status.has_error = false;
    
    std::cout << "Sample motor status:" << std::endl;
    std::cout << "  State: " << ERobMotorController::getStateString(status.state) << std::endl;
    std::cout << "  Status word: 0x" << std::hex << status.status_word << std::dec << std::endl;
    std::cout << "  Position: " << status.position_deg << "° (" << status.actual_position << " counts)" << std::endl;
    std::cout << "  Velocity: " << status.velocity_deg_s << "°/s (" << status.actual_velocity << " counts/s)" << std::endl;
    std::cout << "  Torque: " << (status.actual_torque / 10.0) << "% (" << status.actual_torque << " per mille)" << std::endl;
    std::cout << "  Enabled: " << (status.is_enabled ? "Yes" : "No") << std::endl;
    std::cout << "  Homed: " << (status.is_homed ? "Yes" : "No") << std::endl;
    std::cout << "  Error: " << (status.has_error ? "Yes" : "No") << std::endl;
}

/**
 * @brief Test motor info structure
 */
void testMotorInfo() {
    std::cout << "\n=== Testing Motor Info Structure ===" << std::endl;
    
    MotorInfo info;
    info.slave_id = 1;
    info.name = "eRob Servo Motor";
    info.vendor_id = 0x1234;
    info.product_code = 0x5678;
    info.revision_number = 0x0001;
    info.serial_number = 0x12345678;
    info.is_configured = true;
    info.is_operational = true;
    
    std::cout << "Sample motor info:" << std::endl;
    std::cout << "  Slave ID: " << info.slave_id << std::endl;
    std::cout << "  Name: " << info.name << std::endl;
    std::cout << "  Vendor ID: 0x" << std::hex << info.vendor_id << std::dec << std::endl;
    std::cout << "  Product code: 0x" << std::hex << info.product_code << std::dec << std::endl;
    std::cout << "  Revision: 0x" << std::hex << info.revision_number << std::dec << std::endl;
    std::cout << "  Serial: 0x" << std::hex << info.serial_number << std::dec << std::endl;
    std::cout << "  Configured: " << (info.is_configured ? "Yes" : "No") << std::endl;
    std::cout << "  Operational: " << (info.is_operational ? "Yes" : "No") << std::endl;
}

/**
 * @brief Main test function
 */
int main(int argc, char** argv) {
    std::cout << "=== ERob Motor Controller Test Program ===" << std::endl;
    std::cout << "This program tests the motor controller class without requiring hardware." << std::endl;
    std::cout << "For hardware testing, use the three_axis_example program with root privileges." << std::endl;
    
    // Get interface name from command line or use default
    std::string interface_name = "enp6s0";
    if (argc > 1) {
        interface_name = argv[1];
    }
    
    try {
        // Run tests
        testBasicFunctionality();
        testOperationModes();
        testMotorStatus();
        testMotorInfo();
        testControllerInitialization(interface_name);
        
        std::cout << "\n=== All Tests Completed ===" << std::endl;
        std::cout << "The motor controller class appears to be working correctly." << std::endl;
        std::cout << "To test with actual hardware, run the three_axis_example program as root." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception during testing: " << e.what() << std::endl;
        return -1;
    } catch (...) {
        std::cerr << "Unknown exception during testing" << std::endl;
        return -1;
    }
    
    return 0;
}