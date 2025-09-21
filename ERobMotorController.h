/**
 * @file ERobMotorController.h
 * @brief EtherCAT Motor Controller Class
 * @author Generated based on eRob_CSV.cpp
 * @date 2025-09-21
 */

#ifndef EROB_MOTOR_CONTROLLER_H
#define EROB_MOTOR_CONTROLLER_H

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <memory>
#include "ethercat.h"

// 电机控制模式枚举
enum class MotorControlMode {
    CSV = 9,    // Cyclic Synchronous Velocity
    CSP = 8,    // Cyclic Synchronous Position  
    PP = 1      // Profile Position
};

// 电机状态枚举
enum class MotorState {
    NOT_READY_TO_SWITCH_ON = 0x00,
    SWITCH_ON_DISABLED = 0x40,
    READY_TO_SWITCH_ON = 0x21,
    SWITCHED_ON = 0x23,
    OPERATION_ENABLED = 0x27,
    QUICK_STOP_ACTIVE = 0x07,
    FAULT_REACTION_ACTIVE = 0x0F,
    FAULT = 0x08
};

// RXPDO 结构（发送到从站的控制数据）
typedef struct {
    uint16_t controlword;      // 0x6040:0，16 位
    int32_t target_velocity;   // 0x60FF:0，32 位 (CSV模式使用)
    int32_t target_position;   // 0x607A:0，32 位 (PP模式使用)
    uint8_t mode_of_operation; // 0x6060:0，8 位
    uint8_t padding;           // 8 位对齐填充
} __attribute__((__packed__)) motor_rxpdo_t;

// TXPDO 结构（从从站接收的状态数据）
typedef struct {
    uint16_t statusword;      // 0x6041:0，16 位
    int32_t actual_position;  // 0x6064:0，32 位
    int32_t actual_velocity;  // 0x606C:0，32 位
    int16_t actual_torque;    // 0x6077:0，16 位
} __attribute__((__packed__)) motor_txpdo_t;

// 单个电机信息结构
struct MotorInfo {
    int slave_id;                    // EtherCAT 从站ID
    std::string name;                // 电机名称
    MotorState state;                // 电机状态
    MotorControlMode current_mode;   // 当前控制模式
    bool is_enabled;                 // 是否使能
    
    // 实时状态数据
    int32_t actual_position;         // 实际位置
    int32_t actual_velocity;         // 实际速度
    int16_t actual_torque;          // 实际转矩
    uint16_t status_word;           // 状态字
    
    // 目标值
    int32_t target_position;        // 目标位置
    int32_t target_velocity;        // 目标速度
    
    MotorInfo() : slave_id(0), state(MotorState::NOT_READY_TO_SWITCH_ON), 
                  current_mode(MotorControlMode::CSV), is_enabled(false),
                  actual_position(0), actual_velocity(0), actual_torque(0),
                  status_word(0), target_position(0), target_velocity(0) {}
};

// CSP模式随动控制参数（简化版）
struct CSPFollowParams {
    bool enabled;            // 是否启用随动
    int32_t current_target;  // 当前目标位置
    int32_t segment_base_position; // 每16个周期的基准位置
    int segment_tick;        // 0..15 周期计数
    double step_residual;    // 步长小数残差，用于累积避免取整损失
    double v_cmd;            // 内部指令速度（counts/s），用于限加速度积分

    CSPFollowParams() : enabled(false), current_target(0), segment_base_position(0), segment_tick(0), step_residual(0.0), v_cmd(0.0) {}
};

/**
 * @class ERobMotorController
 * @brief EtherCAT 电机控制器类
 */
class ERobMotorController {
public:
    /**
     * @brief 构造函数
     * @param interface_name EtherCAT网络接口名称（如"enp3s0"）
     * @param cycle_time_us 控制周期时间（微秒），默认1000us
     */
    ERobMotorController(const std::string& interface_name, int cycle_time_us = 1000);
    
    /**
     * @brief 析构函数
     */
    ~ERobMotorController();

    // 基础功能接口
    /**
     * @brief 初始化EtherCAT网络并扫描所有电机
     * @return true 成功，false 失败
     */
    bool initialize();
    
    /**
     * @brief 停止控制并关闭连接
     */
    void shutdown();
    
    /**
     * @brief 获取电机总数量
     * @return 电机数量
     */
    int getMotorCount() const;
    
    /**
     * @brief 获取指定电机的位置
     * @param motor_id 电机ID（从0开始）
     * @return 电机位置（编码器计数）
     */
    int32_t getMotorPosition(int motor_id) const;
    
    /**
     * @brief 获取指定电机的速度
     * @param motor_id 电机ID（从0开始）
     * @return 电机速度（编码器计数/秒）
     */
    int32_t getMotorVelocity(int motor_id) const;
    
    /**
     * @brief 获取指定电机的转矩
     * @param motor_id 电机ID（从0开始）
     * @return 电机转矩
     */
    int16_t getMotorTorque(int motor_id) const;
    
    /**
     * @brief 获取指定电机的状态
     * @param motor_id 电机ID（从0开始）
     * @return 电机状态
     */
    MotorState getMotorState(int motor_id) const;
    
    /**
     * @brief 使能/禁能指定电机
     * @param motor_id 电机ID（从0开始）
     * @param enable true为使能，false为禁能
     * @return true 成功，false 失败
     */
    bool enableMotor(int motor_id, bool enable);
    
    /**
     * @brief 使能/禁能所有电机
     * @param enable true为使能，false为禁能
     * @return true 成功，false 失败
     */
    bool enableAllMotors(bool enable);

    // 单步位置控制接口
    /**
     * @brief 单电机位置到达控制（PP模式）
     * @param motor_id 电机ID（从0开始）
     * @param target_position 目标位置
     * @param velocity 运动速度
     * @param blocking 是否阻塞等待到达
     * @return true 成功，false 失败
     */
    bool moveToPosition(int motor_id, int32_t target_position, int32_t velocity, bool blocking = false);
    
    /**
     * @brief 检查电机是否到达目标位置
     * @param motor_id 电机ID（从0开始）
     * @param tolerance 位置容差
     * @return true 已到达，false 未到达
     */
    bool isMotorAtTarget(int motor_id, int32_t tolerance = 100) const;

    // CSP随动控制接口
    /**
     * @brief 启用/禁用CSP随动模式
     * @param motor_id 电机ID（从0开始）
     * @param enable true为启用，false为禁用
     * @return true 成功，false 失败
     */
    bool enableCSPFollow(int motor_id, bool enable);
    
    /**
     * @brief 设置CSP模式的目标位置
     * @param motor_id 电机ID（从0开始）
     * @param target_position 目标位置
     * @return true 成功，false 失败
     */
    bool setCSPTargetPosition(int motor_id, int32_t target_position);
    
    /**
     * @brief 设置CSP模式的运动参数
     * @param motor_id 电机ID（从0开始）
     * @param max_velocity 最大速度
     * @param max_acceleration 最大加速度
     * @return true 成功，false 失败
     */
    // 已弃用的CSP参数接口移除

    // 实用工具函数
    /**
     * @brief 编码器计数转换为角度
     * @param counts 编码器计数
     * @return 角度（度）
     */
    static double countsToAngle(int32_t counts);
    
    /**
     * @brief 角度转换为编码器计数
     * @param angle 角度（度）
     * @return 编码器计数
     */
    static int32_t angleToCounts(double angle);

private:
    // 私有成员变量
    std::string interface_name_;
    int cycle_time_us_;
    bool initialized_;
    std::atomic<bool> running_;
    
    std::vector<MotorInfo> motors_;
    std::vector<CSPFollowParams> csp_params_;
    
    // EtherCAT 相关
    char io_map_[4096];
    int expected_wkc_;
    volatile int wkc_;
    bool in_op_;
    
    // 线程管理
    std::thread realtime_thread_;
    std::thread check_thread_;
    mutable std::mutex motor_data_mutex_;
    
    // 时钟同步
    int64_t toff_;
    int64_t gl_delta_;
    
    // 私有方法
    bool initializeEtherCAT();
    bool configurePDOMapping();
    bool setupDistributedClock();
    bool transitionToOperational();
    void realtimeLoop();
    void checkLoop();
    
    void updateMotorStatus(int motor_id);
    MotorState parseStatusWord(uint16_t status_word) const;
    uint16_t generateControlWord(MotorState current_state, bool enable) const;
    uint16_t generatePPControlWord(MotorState current_state, bool enable, bool new_setpoint) const;
    
    void ec_sync(int64_t reftime, int64_t cycletime, int64_t* offsettime);
    void add_timespec(struct timespec* ts, int64_t addtime);
    
    bool isValidMotorId(int motor_id) const;
    int32_t calculateCSPInterpolatedPosition(int motor_id);
    
    // 常量定义
    static constexpr double CNT_TO_DEG = 0.000686645;  // 计数到角度转换系数
    static constexpr int32_t MAX_VELOCITY = 50000;     // 最大速度 (counts/s) 与0x6080保持一致
    static constexpr int32_t MAX_ACCELERATION = 130000; // 最大加速度 (counts/s^2) 与0x60C5一致
    static constexpr int NSEC_PER_SEC = 1000000000;    // 每秒纳秒数
};

#endif // EROB_MOTOR_CONTROLLER_H