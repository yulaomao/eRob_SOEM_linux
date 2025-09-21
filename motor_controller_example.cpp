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
#include <random>

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

        // // 示例3: 单步位置控制 (PP模式)
        // if (motor_count > 0) {
        //     std::cout << "\n===== Example 3: Position Control (PP Mode) =====" << std::endl;
            
        //     int motor_id = 0;  // 使用第一个电机
        //     int32_t current_pos = controller.getMotorPosition(motor_id);
        //     int32_t target_pos = current_pos + ERobMotorController::angleToCounts(90.0);  // 移动90度
        //     target_pos = 0;
        //     int32_t velocity = 15000;  // 速度

        //     std::cout << "Moving motor " << motor_id << " from " << current_pos 
        //               << " to " << target_pos << " at velocity " << velocity << std::endl;

        //     if (controller.moveToPosition(motor_id, target_pos, velocity, false)) {
        //         std::cout << "Position command sent successfully" << std::endl;
                
        //         // 等待运动完成
        //         std::cout << "Waiting for movement to complete..." << std::endl;
        //         int timeout = 200;  // 20秒超时
        //         while (timeout-- > 0) {
        //             if (controller.isMotorAtTarget(motor_id, 100)) {
        //                 std::cout << "Motor reached target position!" << std::endl;
        //                 break;
        //             }
        //             std::cout << "Current position: " << controller.getMotorPosition(motor_id) 
        //                       << ", Target: " << target_pos << std::endl;
        //             std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //         }
                
        //         if (timeout <= 0) {
        //             std::cout << "Movement timeout!" << std::endl;
        //         }
        //     }
        // }

        // 示例4: CSP随动控制
        if (motor_count > 0) {
            std::cout << "\n===== Example 4: CSP Follow Control =====" << std::endl;

            int motor_id = 1;  // 使用第二个电机

            // 启用CSP随动模式
            if (controller.enableCSPFollow(motor_id, true)) {
                std::cout << "CSP follow mode enabled for motor " << motor_id << std::endl;
                
                // 获取初始位置
                int32_t initial_pos = controller.getMotorPosition(motor_id);
                std::cout << "Initial position: " << initial_pos << std::endl;
                
                // 随动测试：每16ms根据正弦速度积分出目标位置（v_max=30000），分段随机反向/减速，且限制在±90度范围内
                std::cout << "Executing CSP follow: sine-wave velocity (v_max=30000), random reversals, limited to ±90deg around start..." << std::endl;

                const int update_ms = 16;              // 目标更新周期，与控制器内部16段插补一致
                const double dt = update_ms / 1000.0;  // 每步时间(s)
                const double v_max = 30000.0;          // 最大速度(计数/秒)
                const double pi = 3.14159265358979323846;

                // 期望位置（相对初始位置，单位：计数）及小数残差用于量化
                int64_t pos_des_counts = 0;
                double residual = 0.0;

                // 软限位：初始位置±90度
                const int32_t range_90 = ERobMotorController::angleToCounts(90.0);
                const int32_t soft_min = initial_pos - range_90;
                const int32_t soft_max = initial_pos + range_90;

                // 随机段生成器：半正弦速度段，频繁反向
                std::mt19937 rng(static_cast<unsigned>(std::chrono::steady_clock::now().time_since_epoch().count()));
                std::uniform_real_distribution<double> dur_dist(0.6, 1.8);     // 段时长T[s]
                std::uniform_real_distribution<double> amp_scale_dist(0.5, 1.0); // 幅值比例
                std::bernoulli_distribution reverse_prob(0.7);                 // 70% 概率反向

                // 当前段状态
                double seg_T = dur_dist(rng);
                double seg_A = amp_scale_dist(rng) * v_max;  // 峰值速度
                int seg_dir = 1;  // 初始方向
                auto seg_start = std::chrono::steady_clock::now();

                auto start_time = std::chrono::steady_clock::now();
                const int total_steps = 1500; // 约24秒（1500*16ms）
                for (int i = 0; i < total_steps; i++) {
                    auto target_time = start_time + std::chrono::milliseconds(i * update_ms);
                    std::this_thread::sleep_until(target_time);

                    // 如到达段末，开启新段（优先反向以体现随机减速/反向）
                    double t_seg = std::chrono::duration<double>(std::chrono::steady_clock::now() - seg_start).count();
                    if (t_seg >= seg_T) {
                        // 选择新方向：大概率反向，小概率保持
                        if (reverse_prob(rng)) seg_dir = -seg_dir;
                        // 更新段参数
                        seg_T = dur_dist(rng);
                        seg_A = amp_scale_dist(rng) * v_max;
                        seg_start = std::chrono::steady_clock::now();
                        t_seg = 0.0;
                    }

                    // 半正弦速度：0->A->0，带方向
                    double phase = (t_seg / seg_T);
                    if (phase > 1.0) phase = 1.0; // 保护
                    double v_des = seg_dir * seg_A * std::sin(pi * phase);
                    // 限幅以确保不超过目标最大速度
                    if (v_des > v_max) v_des = v_max;
                    if (v_des < -v_max) v_des = -v_max;

                    // 位置积分（带残差的整数量化）
                    double delta_counts_d = v_des * dt + residual;
                    long delta_counts = static_cast<long>(std::llround(delta_counts_d));
                    residual = delta_counts_d - static_cast<double>(delta_counts);
                    pos_des_counts += delta_counts;

                    // 目标为初始位置 + 期望相对位移，并施加±90度软限位
                    int64_t abs_target = static_cast<int64_t>(initial_pos) + pos_des_counts;
                    bool hit_boundary = false;
                    if (abs_target < soft_min) {
                        abs_target = soft_min;
                        pos_des_counts = static_cast<int64_t>(soft_min) - static_cast<int64_t>(initial_pos);
                        residual = 0.0;
                        seg_dir = +1;   // 反向
                        hit_boundary = true;
                    } else if (abs_target > soft_max) {
                        abs_target = soft_max;
                        pos_des_counts = static_cast<int64_t>(soft_max) - static_cast<int64_t>(initial_pos);
                        residual = 0.0;
                        seg_dir = -1;   // 反向
                        hit_boundary = true;
                    }

                    int32_t target = static_cast<int32_t>(abs_target);
                    controller.setCSPTargetPosition(motor_id, target);

                    if (hit_boundary) {
                        // 到边界后重启一个速度段，避免长时间贴边
                        seg_start = std::chrono::steady_clock::now();
                        seg_T = dur_dist(rng);
                        seg_A = amp_scale_dist(rng) * v_max;
                    }

                    if (i % 20 == 0) {
                        int32_t current = controller.getMotorPosition(motor_id);
                        int32_t velocity = controller.getMotorVelocity(motor_id);
                        std::cout << "Step " << i
                                  << " - v_des: " << static_cast<int>(v_des)
                                  << ", pos_des: " << pos_des_counts
                                  << ", Target: " << target
                                  << ", Current: " << current
                                  << ", Vel(feedback): " << velocity
                                  << std::endl;
                    }
                }

                std::cout << "Follow test completed, holding final position for 2 seconds..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(2));
                
                // 禁用CSP随动模式
                controller.enableCSPFollow(motor_id, false);
                std::cout << "CSP follow mode disabled" << std::endl;
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