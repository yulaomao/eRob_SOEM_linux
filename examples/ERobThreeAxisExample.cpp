/**
 * @file ERobThreeAxisExample.cpp
 * @brief Example implementation demonstrating three-axis robot arm control using ERobMotorController
 * @author eRob SOEM Linux Project
 * @date 2024
 */

#include "ERobMotorController.hpp"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>
#include <iomanip>

/**
 * @brief Three-axis robot arm controller class
 * 
 * This class demonstrates how to use ERobMotorController for coordinated
 * multi-axis motion control typical in robotic arm applications.
 */
class ThreeAxisRobotArm {
public:
    explicit ThreeAxisRobotArm(const std::string& interface_name = "enp6s0") 
        : motor_controller_(interface_name, 1000) // 1ms cycle time
    {
        // Define axis names for clarity
        axis_names_ = {"Base", "Shoulder", "Elbow"};
    }

    /**
     * @brief Initialize the robot arm
     */
    bool initialize() {
        std::cout << "Initializing Three-Axis Robot Arm..." << std::endl;
        
        auto result = motor_controller_.initialize();
        if (result != ControllerError::SUCCESS) {
            std::cerr << "Failed to initialize motor controller: " 
                      << ERobMotorController::getErrorString(result) << std::endl;
            return false;
        }
        
        // Check if we have at least 3 motors
        if (motor_controller_.getMotorCount() < 3) {
            std::cerr << "Error: Need at least 3 motors for three-axis arm, found: " 
                      << motor_controller_.getMotorCount() << std::endl;
            return false;
        }
        
        // Get motor information
        auto motor_info = motor_controller_.getMotorInfo();
        std::cout << "Found " << motor_info.size() << " motors:" << std::endl;
        for (size_t i = 0; i < std::min(motor_info.size(), size_t(3)); ++i) {
            std::cout << "  " << axis_names_[i] << " (Motor " << motor_info[i].slave_id 
                      << "): " << motor_info[i].name << std::endl;
        }
        
        // Set motion parameters for each axis
        MotionParameters base_params;
        base_params.max_velocity = 180.0;      // 180 deg/s
        base_params.max_acceleration = 360.0;  // 360 deg/s²
        base_params.position_tolerance = 0.1;  // 0.1 degree tolerance
        
        MotionParameters shoulder_params;
        shoulder_params.max_velocity = 120.0;
        shoulder_params.max_acceleration = 240.0;
        shoulder_params.position_tolerance = 0.1;
        
        MotionParameters elbow_params;
        elbow_params.max_velocity = 150.0;
        elbow_params.max_acceleration = 300.0;
        elbow_params.position_tolerance = 0.1;
        
        motor_controller_.setMotionParameters(1, base_params);
        motor_controller_.setMotionParameters(2, shoulder_params);
        motor_controller_.setMotionParameters(3, elbow_params);
        
        std::cout << "Three-axis robot arm initialized successfully!" << std::endl;
        return true;
    }
    
    /**
     * @brief Shutdown the robot arm safely
     */
    void shutdown() {
        std::cout << "Shutting down robot arm..." << std::endl;
        motor_controller_.shutdown();
    }
    
    /**
     * @brief Enable all three axes
     */
    bool enableAllAxes() {
        std::cout << "Enabling all axes..." << std::endl;
        
        for (uint16_t i = 1; i <= 3; ++i) {
            auto result = motor_controller_.enableMotor(i);
            if (result != ControllerError::SUCCESS) {
                std::cerr << "Failed to enable " << axis_names_[i-1] << " axis: "
                          << ERobMotorController::getErrorString(result) << std::endl;
                return false;
            }
            std::cout << "  " << axis_names_[i-1] << " axis enabled" << std::endl;
        }
        
        return true;
    }
    
    /**
     * @brief Home all three axes
     */
    bool homeAllAxes() {
        std::cout << "Homing all axes..." << std::endl;
        
        for (uint16_t i = 1; i <= 3; ++i) {
            std::cout << "  Homing " << axis_names_[i-1] << " axis..." << std::endl;
            auto result = motor_controller_.homeMotor(i, 30000); // 30 second timeout
            if (result != ControllerError::SUCCESS) {
                std::cerr << "Failed to home " << axis_names_[i-1] << " axis: "
                          << ERobMotorController::getErrorString(result) << std::endl;
                return false;
            }
            std::cout << "    " << axis_names_[i-1] << " axis homed successfully" << std::endl;
        }
        
        return true;
    }
    
    /**
     * @brief Move to a specific joint configuration
     * @param base_angle Base rotation angle in degrees
     * @param shoulder_angle Shoulder angle in degrees
     * @param elbow_angle Elbow angle in degrees
     * @param wait_for_completion Wait for all axes to reach target
     */
    bool moveToJointAngles(double base_angle, double shoulder_angle, double elbow_angle,
                          bool wait_for_completion = true) {
        std::cout << "Moving to joint angles: Base=" << base_angle 
                  << "°, Shoulder=" << shoulder_angle 
                  << "°, Elbow=" << elbow_angle << "°" << std::endl;
        
        // Move all axes simultaneously
        auto result1 = motor_controller_.moveToPosition(1, base_angle, 0, 0, 0, false);
        auto result2 = motor_controller_.moveToPosition(2, shoulder_angle, 0, 0, 0, false);
        auto result3 = motor_controller_.moveToPosition(3, elbow_angle, 0, 0, 0, false);
        
        if (result1 != ControllerError::SUCCESS || 
            result2 != ControllerError::SUCCESS || 
            result3 != ControllerError::SUCCESS) {
            std::cerr << "Failed to initiate movement" << std::endl;
            return false;
        }
        
        if (wait_for_completion) {
            return waitForAllAxesToReachTarget();
        }
        
        return true;
    }
    
    /**
     * @brief Enable servo mode for smooth trajectory following
     */
    bool enableServoMode() {
        std::cout << "Enabling servo mode for all axes..." << std::endl;
        
        for (uint16_t i = 1; i <= 3; ++i) {
            auto result = motor_controller_.enableServoMode(i);
            if (result != ControllerError::SUCCESS) {
                std::cerr << "Failed to enable servo mode for " << axis_names_[i-1] 
                          << " axis: " << ERobMotorController::getErrorString(result) << std::endl;
                return false;
            }
        }
        
        std::cout << "Servo mode enabled for all axes" << std::endl;
        return true;
    }
    
    /**
     * @brief Perform a circular motion demonstration in servo mode
     */
    bool performCircularMotion(double center_base = 0.0, double center_shoulder = 45.0, 
                              double radius = 15.0, double duration_seconds = 10.0) {
        std::cout << "Performing circular motion demo..." << std::endl;
        
        if (!enableServoMode()) {
            return false;
        }
        
        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + std::chrono::seconds(static_cast<int>(duration_seconds));
        
        while (std::chrono::steady_clock::now() < end_time) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
            double t = elapsed.count() / 1000.0; // Convert to seconds
            
            // Calculate circular trajectory
            double angle = 2.0 * M_PI * t / duration_seconds;
            double base_target = center_base + radius * std::cos(angle);
            double shoulder_target = center_shoulder + radius * std::sin(angle);
            double elbow_target = -shoulder_target; // Keep end-effector roughly horizontal
            
            // Set servo targets
            std::map<uint16_t, double> targets;
            targets[1] = base_target;
            targets[2] = shoulder_target;
            targets[3] = elbow_target;
            
            motor_controller_.setMultipleServoTargets(targets);
            
            // Print current positions every second
            if (static_cast<int>(t) % 1 == 0 && t > static_cast<int>(t) - 0.1) {
                printCurrentPositions();
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20 Hz updates
        }
        
        std::cout << "Circular motion demo completed" << std::endl;
        return true;
    }
    
    /**
     * @brief Emergency stop all axes
     */
    void emergencyStop() {
        std::cout << "EMERGENCY STOP!" << std::endl;
        motor_controller_.emergencyStop();
    }
    
    /**
     * @brief Print current positions of all axes
     */
    void printCurrentPositions() {
        std::cout << "Current positions: ";
        for (uint16_t i = 1; i <= 3; ++i) {
            double position;
            if (motor_controller_.getPosition(i, position) == ControllerError::SUCCESS) {
                std::cout << axis_names_[i-1] << "=" << std::fixed << std::setprecision(2) 
                          << position << "° ";
            }
        }
        std::cout << std::endl;
    }
    
    /**
     * @brief Print detailed status of all axes
     */
    void printDetailedStatus() {
        std::cout << "\n=== Robot Arm Status ===" << std::endl;
        
        for (uint16_t i = 1; i <= 3; ++i) {
            MotorStatus status;
            if (motor_controller_.getMotorStatus(i, status) == ControllerError::SUCCESS) {
                std::cout << axis_names_[i-1] << " Axis (Motor " << i << "):" << std::endl;
                std::cout << "  State: " << ERobMotorController::getStateString(status.state) << std::endl;
                std::cout << "  Position: " << std::fixed << std::setprecision(2) 
                          << status.position_deg << "°" << std::endl;
                std::cout << "  Velocity: " << std::fixed << std::setprecision(2) 
                          << status.velocity_deg_s << "°/s" << std::endl;
                std::cout << "  Torque: " << std::fixed << std::setprecision(1) 
                          << (status.actual_torque / 10.0) << "%" << std::endl;
                std::cout << "  Enabled: " << (status.is_enabled ? "Yes" : "No") << std::endl;
                std::cout << "  Homed: " << (status.is_homed ? "Yes" : "No") << std::endl;
                std::cout << "  Error: " << (status.has_error ? "Yes" : "No") << std::endl;
            }
            std::cout << std::endl;
        }
    }

private:
    ERobMotorController motor_controller_;
    std::vector<std::string> axis_names_;
    
    /**
     * @brief Wait for all axes to reach their target positions
     */
    bool waitForAllAxesToReachTarget(double timeout_seconds = 30.0) {
        std::cout << "Waiting for all axes to reach target..." << std::endl;
        
        auto start_time = std::chrono::steady_clock::now();
        auto timeout_time = start_time + std::chrono::seconds(static_cast<int>(timeout_seconds));
        
        while (std::chrono::steady_clock::now() < timeout_time) {
            bool all_reached = true;
            
            for (uint16_t i = 1; i <= 3; ++i) {
                bool reached;
                if (motor_controller_.isTargetReached(i, reached) != ControllerError::SUCCESS) {
                    return false;
                }
                if (!reached) {
                    all_reached = false;
                    break;
                }
            }
            
            if (all_reached) {
                std::cout << "All axes reached target positions" << std::endl;
                return true;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        std::cerr << "Timeout waiting for axes to reach target" << std::endl;
        return false;
    }
};

/**
 * @brief Main function demonstrating three-axis robot arm control
 */
int main(int argc, char** argv) {
    std::cout << "=== eRob Three-Axis Robot Arm Demo ===" << std::endl;
    
    // Check command line arguments
    std::string interface = "enp6s0"; // Default interface
    if (argc > 1) {
        interface = argv[1];
    }
    
    std::cout << "Using EtherCAT interface: " << interface << std::endl;
    std::cout << "Run as root for EtherCAT access!" << std::endl;
    
    // Create robot arm controller
    ThreeAxisRobotArm robot_arm(interface);
    
    try {
        // Initialize the robot arm
        if (!robot_arm.initialize()) {
            std::cerr << "Failed to initialize robot arm" << std::endl;
            return -1;
        }
        
        // Enable all axes
        if (!robot_arm.enableAllAxes()) {
            std::cerr << "Failed to enable axes" << std::endl;
            return -1;
        }
        
        // Home all axes
        std::cout << "\nDo you want to home all axes? (y/n): ";
        char response;
        std::cin >> response;
        if (response == 'y' || response == 'Y') {
            if (!robot_arm.homeAllAxes()) {
                std::cerr << "Failed to home axes" << std::endl;
                return -1;
            }
        }
        
        // Print initial status
        robot_arm.printDetailedStatus();
        
        // Demonstration sequence
        std::cout << "\n=== Running Demo Sequence ===" << std::endl;
        
        // Move to home position
        std::cout << "\n1. Moving to home position..." << std::endl;
        robot_arm.moveToJointAngles(0.0, 0.0, 0.0, true);
        robot_arm.printCurrentPositions();
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Move to various positions
        std::cout << "\n2. Moving to position 1..." << std::endl;
        robot_arm.moveToJointAngles(45.0, 30.0, -30.0, true);
        robot_arm.printCurrentPositions();
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        std::cout << "\n3. Moving to position 2..." << std::endl;
        robot_arm.moveToJointAngles(-30.0, 60.0, -60.0, true);
        robot_arm.printCurrentPositions();
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Demonstrate servo mode with circular motion
        std::cout << "\n4. Demonstrating servo mode with circular motion..." << std::endl;
        robot_arm.performCircularMotion(0.0, 45.0, 20.0, 8.0);
        
        // Return to home
        std::cout << "\n5. Returning to home position..." << std::endl;
        robot_arm.moveToJointAngles(0.0, 0.0, 0.0, true);
        
        // Final status
        robot_arm.printDetailedStatus();
        
        std::cout << "\n=== Demo Completed Successfully ===" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        robot_arm.emergencyStop();
        return -1;
    }
    
    // Shutdown
    robot_arm.shutdown();
    
    return 0;
}

// Additional utility functions for more complex applications

/**
 * @brief Simple inverse kinematics for demonstration
 * This is a simplified 2D planar arm IK for educational purposes
 */
struct IKResult {
    bool success;
    double base_angle;
    double shoulder_angle;
    double elbow_angle;
};

IKResult simpleInverseKinematics(double x, double y, double z, 
                                double link1_length = 100.0, 
                                double link2_length = 100.0) {
    IKResult result;
    result.success = false;
    
    // Base angle (rotation around Z axis)
    result.base_angle = std::atan2(y, x) * 180.0 / M_PI;
    
    // 2D problem in the plane
    double r = std::sqrt(x*x + y*y);
    double distance = std::sqrt(r*r + z*z);
    
    // Check if target is reachable
    if (distance > (link1_length + link2_length) || 
        distance < std::abs(link1_length - link2_length)) {
        return result;
    }
    
    // Law of cosines
    double cos_elbow = (link1_length*link1_length + link2_length*link2_length - distance*distance) /
                       (2.0 * link1_length * link2_length);
    
    if (std::abs(cos_elbow) > 1.0) {
        return result;
    }
    
    result.elbow_angle = std::acos(cos_elbow) * 180.0 / M_PI - 180.0; // Negative for robot convention
    
    double alpha = std::atan2(z, r);
    double beta = std::acos((link1_length*link1_length + distance*distance - link2_length*link2_length) /
                           (2.0 * link1_length * distance));
    
    result.shoulder_angle = (alpha + beta) * 180.0 / M_PI;
    
    result.success = true;
    return result;
}