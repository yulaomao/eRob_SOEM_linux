/**
 * @file motor_controller_example.cpp
 * @brief ERobMotorController使用示例
 * @author Generated based on eRob_CSV.cpp
 * @date 2025-09-21
 */

#include "ERobMotorController.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <vector>

int main(int argc, char* argv[]) {
    // 检查命令行参数
    std::string interface_name = "enp3s0";  // 默认网络接口
    if (argc > 1) {
        interface_name = argv[1];
    }

    std::cout << "===== ERobMotorController Example =====" << std::endl;
    std::cout << "Using network interface: " << interface_name << std::endl;

    // 创建电机控制器实例
    ERobMotorController controller(interface_name, 1000);  // 1ms控制周期

    // 初始化控制器
    if (!controller.initialize()) {
        std::cerr << "Failed to initialize motor controller!" << std::endl;
        return -1;
    }

    // 获取电机数量
    int motor_count = controller.getMotorCount();
    std::cout << "Found " << motor_count << " motors" << std::endl;

    if (motor_count == 0) {
        std::cout << "No motors found, exiting..." << std::endl;
        return -1;
    }

    try {
        // 等待系统稳定
        std::cout << "\nWaiting for system to stabilize..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // 示例1: 基础状态读取
        std::cout << "\n===== Example 1: Reading Motor Status =====" << std::endl;
        for (int i = 0; i < motor_count; i++) {
            std::cout << "Motor " << i << ":" << std::endl;
            std::cout << "  Position: " << controller.getMotorPosition(i) 
                      << " (" << ERobMotorController::countsToAngle(controller.getMotorPosition(i)) << " deg)" << std::endl;
            std::cout << "  Velocity: " << controller.getMotorVelocity(i) << std::endl;
            std::cout << "  Torque: " << controller.getMotorTorque(i) << std::endl;
            std::cout << "  State: " << static_cast<int>(controller.getMotorState(i)) << std::endl;
        }

        // 示例2: 使能电机
        std::cout << "\n===== Example 2: Enabling Motors =====" << std::endl;
        if (controller.enableAllMotors(true)) {
            std::cout << "All motors enabled successfully" << std::endl;
        } else {
            std::cout << "Failed to enable motors" << std::endl;
            return -1;
        }

        // 等待电机使能完成
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // 示例3: 单步位置控制 (PP模式)
        if (motor_count > 0) {
            std::cout << "\n===== Example 3: Position Control (PP Mode) =====" << std::endl;
            
            int motor_id = 0;  // 使用第一个电机
            int32_t current_pos = controller.getMotorPosition(motor_id);
            int32_t target_pos = current_pos + ERobMotorController::angleToCounts(90.0);  // 移动90度
            target_pos = 0;
            int32_t velocity = 15000;  // 速度

            std::cout << "Moving motor " << motor_id << " from " << current_pos 
                      << " to " << target_pos << " at velocity " << velocity << std::endl;

            if (controller.moveToPosition(motor_id, target_pos, velocity, false)) {
                std::cout << "Position command sent successfully" << std::endl;
                
                // 等待运动完成
                std::cout << "Waiting for movement to complete..." << std::endl;
                int timeout = 200;  // 20秒超时
                while (timeout-- > 0) {
                    if (controller.isMotorAtTarget(motor_id, 100)) {
                        std::cout << "Motor reached target position!" << std::endl;
                        break;
                    }
                    std::cout << "Current position: " << controller.getMotorPosition(motor_id) 
                              << ", Target: " << target_pos << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                
                if (timeout <= 0) {
                    std::cout << "Movement timeout!" << std::endl;
                }
            }
        }

        // 示例4: CSP随动控制
        if (motor_count > 0) {
            std::cout << "\n===== Example 4: CSP Follow Control =====" << std::endl;
            
            int motor_id = 0;  // 使用第一个电机
            
            // 启用CSP随动模式
            if (controller.enableCSPFollow(motor_id, true)) {
                std::cout << "CSP follow mode enabled for motor " << motor_id << std::endl;
                
                // 设置运动参数（适合视觉伺服）
                controller.setCSPMotionParams(motor_id, 15000, 30000);  // 最大速度15000, 最大加速度30000
                
                // 设置平滑参数（减少震动和超调）
                controller.setCSPSmoothParams(motor_id, 100000.0, 0.008, 0.008);  // jerk_limit, acc_time, dec_time
                
                // 获取初始位置
                int32_t initial_pos = controller.getMotorPosition(motor_id);
                std::cout << "Initial position: " << initial_pos << std::endl;
                
                // 模拟视觉伺服场景：16ms周期更新目标位置
                std::cout << "Executing visual servo CSP movements (16ms cycle)..." << std::endl;
                
                // 运动序列: 模拟视觉跟踪目标的运动轨迹
                std::vector<int32_t> target_positions;
                int32_t amplitude = ERobMotorController::angleToCounts(60.0);  // 60度幅度
                
                // 生成正弦波轨迹（模拟目标运动）
                for (int i = 0; i <= 100; i++) {
                    double angle = i * 2.0 * M_PI / 50.0;  // 2个周期
                    int32_t target = initial_pos + static_cast<int32_t>(amplitude * sin(angle));
                    target_positions.push_back(target);
                }
                
                // 执行视觉伺服测试：精确的16ms周期目标更新
                auto start_time = std::chrono::steady_clock::now();
                
                for (size_t i = 0; i < target_positions.size(); i++) {
                    int32_t target = target_positions[i];
                    
                    // 16ms精确周期更新目标位置
                    auto target_time = start_time + std::chrono::milliseconds(i * 16);
                    std::this_thread::sleep_until(target_time);
                    
                    controller.setCSPTargetPosition(motor_id, target);
                    
                    // 每20个点显示一次状态
                    if (i % 20 == 0) {
                        int32_t current = controller.getMotorPosition(motor_id);
                        int32_t velocity = controller.getMotorVelocity(motor_id);
                        std::cout << "Step " << i << " - Target: " << target 
                                  << ", Current: " << current 
                                  << ", Velocity: " << velocity << std::endl;
                    }
                }
                
                std::cout << "Visual servo test completed, holding final position for 2 seconds..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(2));
                
                // 最终状态检查
                int32_t final_pos = controller.getMotorPosition(motor_id);
                int32_t final_target = target_positions.back();
                int32_t final_error = std::abs(final_pos - final_target);
                
                std::cout << "Final position check:" << std::endl;
                std::cout << "  Target: " << final_target << " (" << ERobMotorController::countsToAngle(final_target) << " deg)" << std::endl;
                std::cout << "  Actual: " << final_pos << " (" << ERobMotorController::countsToAngle(final_pos) << " deg)" << std::endl;
                std::cout << "  Error:  " << final_error << " counts (" << ERobMotorController::countsToAngle(final_error) << " deg)" << std::endl;
                
                // 禁用CSP随动模式
                controller.enableCSPFollow(motor_id, false);
                std::cout << "CSP follow mode disabled" << std::endl;
            }
        }

        // 示例5: 多电机协调运动
        if (motor_count > 1) {
            std::cout << "\n===== Example 5: Multi-Motor Coordination =====" << std::endl;
            
            // 同时启用多个电机的CSP模式
            std::vector<int32_t> initial_positions(motor_count);
            for (int i = 0; i < std::min(motor_count, 3); i++) {  // 最多控制3个电机
                initial_positions[i] = controller.getMotorPosition(i);
                controller.enableCSPFollow(i, true);
                controller.setCSPMotionParams(i, 1500, 2000);
                std::cout << "Motor " << i << " initial position: " << initial_positions[i] << std::endl;
            }
            
            // 协调运动: 所有电机同时运动到不同位置
            std::cout << "Coordinated movement started..." << std::endl;
            for (int i = 0; i < std::min(motor_count, 3); i++) {
                int32_t target = initial_positions[i] + ERobMotorController::angleToCounts(30.0 * (i + 1));
                controller.setCSPTargetPosition(i, target);
                std::cout << "Motor " << i << " target set to: " << target << std::endl;
            }
            
            // 监控运动过程
            for (int step = 0; step < 100; step++) {  // 10秒监控
                std::cout << "Step " << step << ": ";
                bool all_close = true;
                
                for (int i = 0; i < std::min(motor_count, 3); i++) {
                    int32_t current = controller.getMotorPosition(i);
                    int32_t target = initial_positions[i] + ERobMotorController::angleToCounts(30.0 * (i + 1));
                    int32_t error = std::abs(current - target);
                    
                    std::cout << "M" << i << ":" << current << "(" << error << ") ";
                    
                    if (error > 200) all_close = false;
                }
                std::cout << std::endl;
                
                if (all_close) {
                    std::cout << "All motors reached targets!" << std::endl;
                    break;
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            // 禁用CSP模式
            for (int i = 0; i < std::min(motor_count, 3); i++) {
                controller.enableCSPFollow(i, false);
            }
        }

        std::cout << "\n===== Example Completed Successfully =====" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
    }

    // 禁能所有电机
    std::cout << "\nDisabling all motors..." << std::endl;
    controller.enableAllMotors(false);
    
    // 等待一段时间让系统清理
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    std::cout << "Shutting down controller..." << std::endl;
    // 析构函数会自动调用shutdown()

    return 0;
}