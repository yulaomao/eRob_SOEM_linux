/**
 * @file ERobMotorController.cpp
 * @brief EtherCAT Motor Controller Class Implementation
 * @author Generated based on eRob_CSV.cpp
 * @date 2025-09-21
 */

#include "ERobMotorController.h"
#include <unistd.h>
#include <sys/mman.h>
#include <sched.h>
#include <algorithm>
#include <cmath>

// 定义静态常量
constexpr double ERobMotorController::CNT_TO_DEG;
constexpr int32_t ERobMotorController::MAX_VELOCITY;
constexpr int32_t ERobMotorController::MAX_ACCELERATION;
constexpr int ERobMotorController::NSEC_PER_SEC;

ERobMotorController::ERobMotorController(const std::string& interface_name, int cycle_time_us)
    : interface_name_(interface_name)
    , cycle_time_us_(cycle_time_us)
    , initialized_(false)
    , running_(false)
    , expected_wkc_(0)
    , wkc_(0)
    , in_op_(false)
    , toff_(0)
    , gl_delta_(0) {
    memset(io_map_, 0, sizeof(io_map_));
}

ERobMotorController::~ERobMotorController() {
    shutdown();
}

bool ERobMotorController::initialize() {
    if (initialized_) {
        std::cout << "Motor controller already initialized" << std::endl;
        return true;
    }

    // 初始化EtherCAT网络
    if (!initializeEtherCAT()) {
        std::cout << "Failed to initialize EtherCAT" << std::endl;
        return false;
    }

    // 配置PDO映射
    if (!configurePDOMapping()) {
        std::cout << "Failed to configure PDO mapping" << std::endl;
        return false;
    }

    // 配置分布式时钟
    if (!setupDistributedClock()) {
        std::cout << "Failed to setup distributed clock" << std::endl;
        return false;
    }

    // 转换到运行状态
    if (!transitionToOperational()) {
        std::cout << "Failed to transition to operational state" << std::endl;
        return false;
    }

    // 初始化电机信息
    motors_.resize(ec_slavecount);
    csp_params_.resize(ec_slavecount);

    for (int i = 0; i < ec_slavecount; i++) {
        motors_[i].slave_id = i + 1;  // EtherCAT从站ID从1开始
        motors_[i].name = ec_slave[i + 1].name;
        motors_[i].state = MotorState::NOT_READY_TO_SWITCH_ON;
        motors_[i].current_mode = MotorControlMode::CSV;
        motors_[i].is_enabled = false;

        // 初始化CSP参数（简化）
        csp_params_[i].enabled = false;
        csp_params_[i].current_target = 0;
    }

    // 启动实时线程
    running_ = true;
    realtime_thread_ = std::thread(&ERobMotorController::realtimeLoop, this);
    check_thread_ = std::thread(&ERobMotorController::checkLoop, this);
    // 简化随动后不再需要独立随动线程

    initialized_ = true;
    std::cout << "Motor controller initialized successfully with " << ec_slavecount << " motors" << std::endl;
    
    return true;
}

void ERobMotorController::shutdown() {
    if (!initialized_) return;

    running_ = false;
    
    // 等待线程结束
    if (realtime_thread_.joinable()) {
        realtime_thread_.join();
    }
    if (check_thread_.joinable()) {
        check_thread_.join();
    }
    // 无随动线程需回收

    // 禁能所有电机
    enableAllMotors(false);

    // 关闭EtherCAT连接
    if (initialized_) {
        ec_slave[0].state = EC_STATE_INIT;
        ec_writestate(0);
        ec_close();
    }

    initialized_ = false;
    std::cout << "Motor controller shutdown completed" << std::endl;
}

bool ERobMotorController::initializeEtherCAT() {
    // 初始化EtherCAT主站
    if (ec_init(interface_name_.c_str()) <= 0) {
        std::cout << "Failed to initialize EtherCAT master on interface: " << interface_name_ << std::endl;
        return false;
    }

    // 搜索EtherCAT从站
    if (ec_config_init(FALSE) <= 0) {
        std::cout << "No EtherCAT slaves found" << std::endl;
        ec_close();
        return false;
    }

    std::cout << "Found " << ec_slavecount << " EtherCAT slaves" << std::endl;

    // 检查从站状态并切换到PRE-OP
    ec_readstate();
    for (int i = 1; i <= ec_slavecount; i++) {
        if (ec_slave[i].state != EC_STATE_PRE_OP) {
            ec_slave[i].state = EC_STATE_INIT;
        } else {
            ec_slave[0].state = EC_STATE_PRE_OP;
            ec_writestate(0);
            if (ec_statecheck(0, EC_STATE_PRE_OP, 3 * EC_TIMEOUTSTATE) != EC_STATE_PRE_OP) {
                std::cout << "Failed to transition to PRE-OP state" << std::endl;
                return false;
            }
        }
    }

    return true;
}

bool ERobMotorController::configurePDOMapping() {
    int retval = 0;
    uint16_t map_val;
    uint8_t zero_map = 0;
    uint32_t map_object;
    uint16_t clear_val = 0x0000;

    // 配置RXPDO映射
    for (int i = 1; i <= ec_slavecount; i++) {
        // 清除RXPDO映射
        retval += ec_SDOwrite(i, 0x1600, 0x00, FALSE, sizeof(zero_map), &zero_map, EC_TIMEOUTSAFE);
        
        // 控制字 (0x6040:0, 16 bits)
        map_object = 0x60400010;
        retval += ec_SDOwrite(i, 0x1600, 0x01, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // 目标速度 (0x60FF:0, 32 bits) - CSV模式
        map_object = 0x60FF0020;
        retval += ec_SDOwrite(i, 0x1600, 0x02, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // 目标位置 (0x607A:0, 32 bits) - PP模式
        map_object = 0x607A0020;
        retval += ec_SDOwrite(i, 0x1600, 0x03, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // 操作模式 (0x6060:0, 8 bits)
        map_object = 0x60600008;
        retval += ec_SDOwrite(i, 0x1600, 0x04, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // 填充 (8 bits)
        map_object = 0x00000008;
        retval += ec_SDOwrite(i, 0x1600, 0x05, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        uint8_t map_count = 5;
        retval += ec_SDOwrite(i, 0x1600, 0x00, FALSE, sizeof(map_count), &map_count, EC_TIMEOUTSAFE);
        
        // 配置RXPDO分配
        clear_val = 0x0000;
        retval += ec_SDOwrite(i, 0x1c12, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);
        map_val = 0x1600;
        retval += ec_SDOwrite(i, 0x1c12, 0x01, FALSE, sizeof(map_val), &map_val, EC_TIMEOUTSAFE);
        map_val = 0x0001;
        retval += ec_SDOwrite(i, 0x1c12, 0x00, FALSE, sizeof(map_val), &map_val, EC_TIMEOUTSAFE);
    }

    if (retval < 0) {
        std::cout << "RXPDO mapping failed" << std::endl;
        return false;
    }

    // 配置TXPDO映射
    retval = 0;
    for (int i = 1; i <= ec_slavecount; i++) {
        // 清除TXPDO映射
        clear_val = 0x0000;
        retval += ec_SDOwrite(i, 0x1A00, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);

        // 状态字 (0x6041:0, 16 bits)
        map_object = 0x60410010;
        retval += ec_SDOwrite(i, 0x1A00, 0x01, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        // 实际位置 (0x6064:0, 32 bits)
        map_object = 0x60640020;
        retval += ec_SDOwrite(i, 0x1A00, 0x02, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        // 实际速度 (0x606C:0, 32 bits)
        map_object = 0x606C0020;
        retval += ec_SDOwrite(i, 0x1A00, 0x03, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        // 实际转矩 (0x6077:0, 16 bits)
        map_object = 0x60770010;
        retval += ec_SDOwrite(i, 0x1A00, 0x04, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        uint8_t map_count = 4;
        retval += ec_SDOwrite(i, 0x1A00, 0x00, FALSE, sizeof(map_count), &map_count, EC_TIMEOUTSAFE);

        // 配置TXPDO分配
        clear_val = 0x0000;
        retval += ec_SDOwrite(i, 0x1C13, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);
        map_val = 0x1A00;
        retval += ec_SDOwrite(i, 0x1C13, 0x01, FALSE, sizeof(map_val), &map_val, EC_TIMEOUTSAFE);
        map_val = 0x0001;
        retval += ec_SDOwrite(i, 0x1C13, 0x00, FALSE, sizeof(map_val), &map_val, EC_TIMEOUTSAFE);
    }

    if (retval < 0) {
        std::cout << "TXPDO mapping failed" << std::endl;
        return false;
    }

    std::cout << "PDO mapping configured successfully" << std::endl;
    return true;
}

bool ERobMotorController::setupDistributedClock() {
    // 禁用手动状态改变
    ecx_context.manualstatechange = 1;
    usleep(1000000);  // 1秒延迟

    // 映射配置的PDO到IOmap
    ec_config_map(&io_map_);

    // 确保所有从站都在PRE-OP状态
    ec_readstate();
    for (int i = 1; i <= ec_slavecount; i++) {
        if (ec_slave[i].state != EC_STATE_PRE_OP) {
            std::cout << "Slave " << i << " not in PRE-OP state" << std::endl;
            return false;
        }
        // 延迟启用Sync0直到OP状态
        ecx_dcsync0(&ecx_context, i, FALSE, 0, 0);
    }

    // 配置分布式时钟
    std::cout << "Configuring distributed clock..." << std::endl;
    ec_configdc();
    usleep(200000);  // 200ms延迟

    return true;
}

bool ERobMotorController::transitionToOperational() {
    // 切换到SAFE-OP状态
    std::cout << "Transitioning to SAFE-OP state..." << std::endl;
    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);
    usleep(200000);

    // 检查状态转换结果
    int timeout_count = 40;
    do {
        ec_readstate();
        bool all_safe_op = true;
        for (int i = 1; i <= ec_slavecount; i++) {
            if (ec_slave[i].state != EC_STATE_SAFE_OP) {
                all_safe_op = false;
                break;
            }
        }
        if (all_safe_op) break;
        usleep(100000);
    } while (timeout_count--);

    if (ec_slave[0].state != EC_STATE_SAFE_OP) {
        std::cout << "Failed to reach SAFE-OP state" << std::endl;
        return false;
    }

    // 计算期望的工作计数器
    expected_wkc_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    std::cout << "Expected work counter: " << expected_wkc_ << std::endl;

    // 在SAFE_OP状态下预配置操作模式与基础SDO参数
    uint8_t operation_mode_csv = 9;  // CSV模式
    for (int i = 1; i <= ec_slavecount; i++) {
        int ret = ec_SDOwrite(i, 0x6060, 0x00, FALSE, sizeof(operation_mode_csv), &operation_mode_csv, EC_TIMEOUTSAFE);
        if (ret <= 0) {
            std::cout << "Warning: Failed to preset operation mode for slave " << i << std::endl;
        }

        // 基础限制与PP默认参数
        int32_t max_velocity = 50000;      // 0x6080: Max profile velocity
        int32_t max_acceleration = 130000;  // 0x60C5: Max acceleration
        int32_t prof_velocity = 15000;     // 0x6081: Profile velocity
        int32_t prof_accel = 10000;        // 0x6083: Profile acceleration
        int32_t prof_decel = 10000;        // 0x6084: Profile deceleration

        ret = ec_SDOwrite(i, 0x6080, 0x00, FALSE, sizeof(max_velocity), &max_velocity, EC_TIMEOUTSAFE);
        if (ret <= 0) {
            std::cout << "Warning: Failed to write 0x6080 (Max velocity) for slave " << i << std::endl;
        }
        ret = ec_SDOwrite(i, 0x60C5, 0x00, FALSE, sizeof(max_acceleration), &max_acceleration, EC_TIMEOUTSAFE);
        if (ret <= 0) {
            std::cout << "Warning: Failed to write 0x60C5 (Max acceleration) for slave " << i << std::endl;
        }
        ret = ec_SDOwrite(i, 0x6081, 0x00, FALSE, sizeof(prof_velocity), &prof_velocity, EC_TIMEOUTSAFE);
        if (ret <= 0) {
            std::cout << "Warning: Failed to write 0x6081 (Profile velocity) for slave " << i << std::endl;
        }
        ret = ec_SDOwrite(i, 0x6083, 0x00, FALSE, sizeof(prof_accel), &prof_accel, EC_TIMEOUTSAFE);
        if (ret <= 0) {
            std::cout << "Warning: Failed to write 0x6083 (Profile acceleration) for slave " << i << std::endl;
        }
        ret = ec_SDOwrite(i, 0x6084, 0x00, FALSE, sizeof(prof_decel), &prof_decel, EC_TIMEOUTSAFE);
        if (ret <= 0) {
            std::cout << "Warning: Failed to write 0x6084 (Profile deceleration) for slave " << i << std::endl;
        }
    }

    // 初始化PDO数据
    motor_rxpdo_t rxpdo;
    rxpdo.controlword = 0x0000;
    rxpdo.target_velocity = 0;
    rxpdo.target_position = 0;
    rxpdo.mode_of_operation = 9;  // CSV模式
    rxpdo.padding = 0;

    // 发送初始PDO数据
    for (int slave = 1; slave <= ec_slavecount; slave++) {
        memcpy(ec_slave[slave].outputs, &rxpdo, sizeof(motor_rxpdo_t));
    }

    // 发送几次数据确保从站准备好
    for (int i = 0; i < 5; i++) {
        ec_send_processdata();
        wkc_ = ec_receive_processdata(EC_TIMEOUTRET);
        usleep(10000);  // 10ms延迟
    }

    // 切换到运行状态
    std::cout << "Transitioning to OPERATIONAL state..." << std::endl;
    
    // 为支持DC的从站启用Sync0
    for (int i = 1; i <= ec_slavecount; i++) {
        if (ec_slave[i].hasdc) {
            ecx_dcsync0(&ecx_context, i, TRUE, 1000000, 0);
        }
    }
    
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);

    // 等待状态转换，同时持续PDO交换
    timeout_count = 2500;  // ~5秒超时
    while (timeout_count > 0) {
        ec_send_processdata();
        wkc_ = ec_receive_processdata(EC_TIMEOUTRET);

        ec_readstate();
        bool all_operational = true;
        for (int i = 1; i <= ec_slavecount; i++) {
            if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                all_operational = false;
                break;
            }
        }
        
        if (all_operational) {
            std::cout << "All slaves reached OPERATIONAL state" << std::endl;
            break;
        }
        
        if ((timeout_count % 100) == 0) {
            // 定期重新请求OP状态
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            ec_writestate(0);
        }
        
        timeout_count--;
        usleep(1000);  // 1ms周期
    }

    if (timeout_count <= 0) {
        std::cout << "ERROR: Failed to reach OPERATIONAL state within timeout" << std::endl;
        return false;
    }

    in_op_ = true;

    // 只有在达到OP状态后才启用Sync0
    for (int i = 1; i <= ec_slavecount; i++) {
        if (ec_slave[i].hasdc) {
            ecx_dcsync0(&ecx_context, i, TRUE, 1000000, 0);
        }
    }

    std::cout << "Successfully transitioned to OPERATIONAL state" << std::endl;
    return true;
}

// 基础功能接口实现
int ERobMotorController::getMotorCount() const {
    return static_cast<int>(motors_.size());
}

int32_t ERobMotorController::getMotorPosition(int motor_id) const {
    if (!isValidMotorId(motor_id)) return 0;
    
    std::lock_guard<std::mutex> lock(motor_data_mutex_);
    return motors_[motor_id].actual_position;
}

int32_t ERobMotorController::getMotorVelocity(int motor_id) const {
    if (!isValidMotorId(motor_id)) return 0;
    
    std::lock_guard<std::mutex> lock(motor_data_mutex_);
    return motors_[motor_id].actual_velocity;
}

int16_t ERobMotorController::getMotorTorque(int motor_id) const {
    if (!isValidMotorId(motor_id)) return 0;
    
    std::lock_guard<std::mutex> lock(motor_data_mutex_);
    return motors_[motor_id].actual_torque;
}

MotorState ERobMotorController::getMotorState(int motor_id) const {
    if (!isValidMotorId(motor_id)) return MotorState::NOT_READY_TO_SWITCH_ON;
    
    std::lock_guard<std::mutex> lock(motor_data_mutex_);
    return motors_[motor_id].state;
}

bool ERobMotorController::enableMotor(int motor_id, bool enable) {
    if (!isValidMotorId(motor_id) || !initialized_) return false;

    std::lock_guard<std::mutex> lock(motor_data_mutex_);
    motors_[motor_id].is_enabled = enable;
    
    std::cout << "Motor " << motor_id << " " << (enable ? "enabled" : "disabled") << std::endl;
    return true;
}

bool ERobMotorController::enableAllMotors(bool enable) {
    if (!initialized_) return false;

    std::lock_guard<std::mutex> lock(motor_data_mutex_);
    for (auto& motor : motors_) {
        motor.is_enabled = enable;
    }
    
    std::cout << "All motors " << (enable ? "enabled" : "disabled") << std::endl;
    return true;
}

// 工具函数
double ERobMotorController::countsToAngle(int32_t counts) {
    return counts * CNT_TO_DEG;
}

int32_t ERobMotorController::angleToCounts(double angle) {
    return static_cast<int32_t>(angle / CNT_TO_DEG);
}

bool ERobMotorController::isValidMotorId(int motor_id) const {
    return motor_id >= 0 && motor_id < static_cast<int>(motors_.size());
}

MotorState ERobMotorController::parseStatusWord(uint16_t status_word) const {
    uint16_t state_mask = status_word & 0x6F;  // 提取状态位
    
    switch (state_mask) {
        case 0x00: return MotorState::NOT_READY_TO_SWITCH_ON;
        case 0x40: return MotorState::SWITCH_ON_DISABLED;
        case 0x21: return MotorState::READY_TO_SWITCH_ON;
        case 0x23: return MotorState::SWITCHED_ON;
        case 0x27: return MotorState::OPERATION_ENABLED;
        case 0x07: return MotorState::QUICK_STOP_ACTIVE;
        case 0x0F: return MotorState::FAULT_REACTION_ACTIVE;
        case 0x08: return MotorState::FAULT;
        default: return MotorState::NOT_READY_TO_SWITCH_ON;
    }
}

uint16_t ERobMotorController::generateControlWord(MotorState current_state, bool enable) const {
    if (!enable) {
        return 0x0006;  // 禁能命令
    }

    switch (current_state) {
        case MotorState::NOT_READY_TO_SWITCH_ON:
        case MotorState::SWITCH_ON_DISABLED:
            return 0x0006;  // Shutdown
        case MotorState::READY_TO_SWITCH_ON:
            return 0x0007;  // Switch on
        case MotorState::SWITCHED_ON:
            return 0x000F;  // Enable operation
        case MotorState::OPERATION_ENABLED:
            return 0x000F;  // 保持使能
        case MotorState::QUICK_STOP_ACTIVE:
            return 0x000F;  // Enable operation
        case MotorState::FAULT:
            return 0x0080;  // Fault reset
        default:
            return 0x0006;  // 默认禁能
    }
}

uint16_t ERobMotorController::generatePPControlWord(MotorState current_state, bool enable, bool new_setpoint) const {
    if (!enable) {
        return 0x0006;  // 禁能命令
    }

    uint16_t control_word = 0x000F;  // 基础使能命令
    
    switch (current_state) {
        case MotorState::NOT_READY_TO_SWITCH_ON:
        case MotorState::SWITCH_ON_DISABLED:
            return 0x0006;  // Shutdown
        case MotorState::READY_TO_SWITCH_ON:
            return 0x0007;  // Switch on
        case MotorState::SWITCHED_ON:
            return 0x000F;  // Enable operation
        case MotorState::OPERATION_ENABLED:
            control_word = 0x000F;  // Enable operation
            if (new_setpoint) {
                control_word |= 0x0010;  // 设置 bit 4 (new setpoint)
            }
            return control_word;
        case MotorState::QUICK_STOP_ACTIVE:
            return 0x000F;  // Enable operation
        case MotorState::FAULT:
            return 0x0080;  // Fault reset
        default:
            return 0x0006;  // 默认禁能
    }
}

void ERobMotorController::updateMotorStatus(int motor_id) {
    if (!isValidMotorId(motor_id)) return;
    
    int slave_id = motor_id + 1;  // EtherCAT从站ID从1开始
    
    // 从TXPDO读取数据
    motor_txpdo_t* txpdo = reinterpret_cast<motor_txpdo_t*>(ec_slave[slave_id].inputs);
    
    if (txpdo) {
        motors_[motor_id].status_word = txpdo->statusword;
        motors_[motor_id].actual_position = txpdo->actual_position;
        motors_[motor_id].actual_velocity = txpdo->actual_velocity;
        motors_[motor_id].actual_torque = txpdo->actual_torque;
        motors_[motor_id].state = parseStatusWord(txpdo->statusword);
    }
}

// 单步位置控制实现
bool ERobMotorController::moveToPosition(int motor_id, int32_t target_position, int32_t velocity, bool blocking) {
    if (!isValidMotorId(motor_id) || !initialized_) return false;

    int slave_id = motor_id + 1;
    
    // 首先确保电机使能
    if (!motors_[motor_id].is_enabled) {
        std::cout << "Motor " << motor_id << " is not enabled, enabling first..." << std::endl;
        enableMotor(motor_id, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 等待使能完成
    }
    
    // 切换到PP模式
    uint8_t pp_mode = static_cast<uint8_t>(MotorControlMode::PP);
    if (ec_SDOwrite(slave_id, 0x6060, 0x00, FALSE, sizeof(pp_mode), &pp_mode, EC_TIMEOUTSAFE) <= 0) {
        std::cout << "Failed to set PP mode for motor " << motor_id << std::endl;
        return false;
    }
    
    // 等待模式切换完成
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 验证模式是否设置成功
    uint8_t actual_mode;
    int size = sizeof(actual_mode);
    if (ec_SDOread(slave_id, 0x6061, 0x00, FALSE, &size, &actual_mode, EC_TIMEOUTSAFE) > 0) {
        std::cout << "Motor " << motor_id << " actual mode: " << (int)actual_mode << std::endl;
        if (actual_mode != pp_mode) {
            std::cout << "Warning: Mode not switched correctly" << std::endl;
        }
    }
    
    // 设置profile velocity (0x6081)
    if (ec_SDOwrite(slave_id, 0x6081, 0x00, FALSE, sizeof(velocity), &velocity, EC_TIMEOUTSAFE) <= 0) {
        std::cout << "Failed to set profile velocity for motor " << motor_id << std::endl;
        return false;
    }
    
    // 设置profile acceleration (0x6083)
    int32_t acceleration = 10000;  // 设置合理的加速度
    if (ec_SDOwrite(slave_id, 0x6083, 0x00, FALSE, sizeof(acceleration), &acceleration, EC_TIMEOUTSAFE) <= 0) {
        std::cout << "Warning: Failed to set profile acceleration for motor " << motor_id << std::endl;
    }
    
    // 设置profile deceleration (0x6084)
    int32_t deceleration = 10000;  // 设置合理的减速度
    if (ec_SDOwrite(slave_id, 0x6084, 0x00, FALSE, sizeof(deceleration), &deceleration, EC_TIMEOUTSAFE) <= 0) {
        std::cout << "Warning: Failed to set profile deceleration for motor " << motor_id << std::endl;
    }
    
    // 设置目标位置 (0x607A)
    if (ec_SDOwrite(slave_id, 0x607A, 0x00, FALSE, sizeof(target_position), &target_position, EC_TIMEOUTSAFE) <= 0) {
        std::cout << "Failed to set target position for motor " << motor_id << std::endl;
        return false;
    }
    
    // 更新内部状态
    {
        std::lock_guard<std::mutex> lock(motor_data_mutex_);
        motors_[motor_id].current_mode = MotorControlMode::PP;
        motors_[motor_id].target_position = target_position;
        motors_[motor_id].target_velocity = velocity;
    }
    
    std::cout << "Motor " << motor_id << " configured for PP mode - target: " << target_position 
              << ", velocity: " << velocity << std::endl;
    
    // 发送启动运动的控制字序列
    // 在PP模式下，需要通过控制字来触发运动
    std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 短暂等待配置生效

    if (blocking) {
        // 阻塞等待到达目标位置
        const int timeout_ms = 10000;  // 10秒超时
        const int check_interval_ms = 10;
        int elapsed_ms = 0;
        
        while (elapsed_ms < timeout_ms) {
            if (isMotorAtTarget(motor_id, 100)) {
                std::cout << "Motor " << motor_id << " reached target position" << std::endl;
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_ms));
            elapsed_ms += check_interval_ms;
        }
        
        std::cout << "Motor " << motor_id << " movement timeout" << std::endl;
        return false;
    }

    return true;
}

bool ERobMotorController::isMotorAtTarget(int motor_id, int32_t tolerance) const {
    if (!isValidMotorId(motor_id)) return false;
    
    std::lock_guard<std::mutex> lock(motor_data_mutex_);
    int32_t position_error = std::abs(motors_[motor_id].actual_position - motors_[motor_id].target_position);
    return position_error <= tolerance;
}

// CSP随动控制实现
bool ERobMotorController::enableCSPFollow(int motor_id, bool enable) {
    if (!isValidMotorId(motor_id) || !initialized_) return false;

    std::lock_guard<std::mutex> lock(motor_data_mutex_);
    
    if (enable) {
        // 切换到CSP模式
        motors_[motor_id].current_mode = MotorControlMode::CSP;
        
    // 初始化CSP参数（简化）
    auto& csp = csp_params_[motor_id];
    csp.enabled = true;
    csp.current_target = motors_[motor_id].actual_position;
    csp.segment_base_position = motors_[motor_id].actual_position;
    csp.segment_tick = 0;
    csp.step_residual = 0.0;
    csp.v_cmd = 0.0;
        
        // 配置CSP模式
        int slave_id = motor_id + 1;
        uint8_t csp_mode = static_cast<uint8_t>(MotorControlMode::CSP);
        if (ec_SDOwrite(slave_id, 0x6060, 0x00, FALSE, sizeof(csp_mode), &csp_mode, EC_TIMEOUTSAFE) <= 0) {
            std::cout << "Failed to set CSP mode for motor " << motor_id << std::endl;
            csp.enabled = false;
            return false;
        }
        
    std::cout << "CSP follow mode enabled for motor " << motor_id 
          << ", initial position: " << motors_[motor_id].actual_position << std::endl;
    } else {
        csp_params_[motor_id].enabled = false;
        std::cout << "CSP follow mode disabled for motor " << motor_id << std::endl;
    }

    return true;
}

bool ERobMotorController::setCSPTargetPosition(int motor_id, int32_t target_position) {
    if (!isValidMotorId(motor_id) || !csp_params_[motor_id].enabled) return false;

    std::lock_guard<std::mutex> lock(motor_data_mutex_);
    csp_params_[motor_id].current_target = target_position;
    return true;
}

int32_t ERobMotorController::calculateCSPInterpolatedPosition(int motor_id) {
    if (!isValidMotorId(motor_id) || !csp_params_[motor_id].enabled) {
        return motors_[motor_id].actual_position;
    }

    // 周期时长（秒）
    const double dt = 1;

    int32_t current = motors_[motor_id].actual_position;
    int32_t target = csp_params_[motor_id].current_target;

    // 每16个周期锁定一次基准位置（segment base）
    auto &csp = csp_params_[motor_id];
    if (csp.segment_tick % 16 == 0) {
        csp.segment_base_position = current;
    }

    int32_t diff = target - csp.segment_base_position;
    double raw_step_d = static_cast<double>(diff) / 16.0; // 未限幅的均分步长（浮点）

    // 基于当前速度和加速度限制计算正/负方向的最大步长
    const double v_cur = static_cast<double>(motors_[motor_id].actual_velocity); // counts/s
    double v_inc_max = std::min(static_cast<double>(MAX_VELOCITY), v_cur + static_cast<double>(MAX_ACCELERATION) * dt);
    double v_dec_min = std::max(-static_cast<double>(MAX_VELOCITY), v_cur - static_cast<double>(MAX_ACCELERATION) * dt);
    double max_step_pos = v_inc_max * dt; // 本周期向正方向最大位移
    double max_step_neg = v_dec_min * dt; // 本周期向负方向最大位移（为负数）

    // 先按均分步长做限幅，再叠加小数残差以实现逐步累积
    double limited_step_d = raw_step_d;
    bool clamped = false;
    if (limited_step_d > max_step_pos) { limited_step_d = max_step_pos; clamped = true; }
    if (limited_step_d < max_step_neg) { limited_step_d = max_step_neg; clamped = true; }
    double cmd_step_d = limited_step_d + csp.step_residual;

    // 量化到整数步长并更新残差
    int32_t step = (cmd_step_d >= 0.0) ? static_cast<int32_t>(std::floor(cmd_step_d))
                                       : static_cast<int32_t>(std::ceil(cmd_step_d));
    csp.step_residual = cmd_step_d - static_cast<double>(step);

    // 不超过均分步长幅值，避免超过16段目标
    if (std::abs(step) > std::abs(static_cast<int32_t>(std::round(raw_step_d)))) {
        step = (raw_step_d > 0.0) ? std::abs(static_cast<int32_t>(std::round(raw_step_d)))
                                  : -std::abs(static_cast<int32_t>(std::round(raw_step_d)));
    }

    // clamped 已在限幅时标记

    // // 调试：段起点打印
    // if (csp.segment_tick % 16 == 0) {
    //     std::cout << "[CSP] motor=" << motor_id
    //               << " seg_tick=" << csp.segment_tick
    //               << " dt(s)=" << dt
    //               << " cur=" << current
    //               << " tgt=" << target
    //               << " base=" << csp.segment_base_position
    //               << " diff=" << diff
    //               << " raw_step_d=" << raw_step_d
    //               << " v_cur=" << v_cur
    //               << " max_step_pos=" << max_step_pos
    //               << " max_step_neg=" << max_step_neg
    //               << " cmd_step_d=" << cmd_step_d
    //               << " residual=" << csp.step_residual
    //               << " step(out)=" << step
    //               << " vel(tx)=" << motors_[motor_id].actual_velocity
    //               << (clamped ? " CLAMPED" : "")
    //               << std::endl;
    // }
    // // 调试：发生限幅时也打印
    // if (clamped && (csp.segment_tick % 16 != 0)) {
    //     std::cout << "[CSP] motor=" << motor_id
    //               << " tick=" << csp.segment_tick
    //               << " cur=" << current
    //               << " tgt=" << target
    //               << " base=" << csp.segment_base_position
    //               << " diff=" << diff
    //               << " raw_step_d=" << raw_step_d
    //               << " v_cur=" << v_cur
    //               << " max_step_pos=" << max_step_pos
    //               << " max_step_neg=" << max_step_neg
    //               << " cmd_step_d=" << cmd_step_d
    //               << " residual=" << csp.step_residual
    //               << " step(out)=" << step
    //               << " vel(tx)=" << motors_[motor_id].actual_velocity
    //               << " CLAMPED"
    //               << std::endl;
    // }

    // 递增tick
    csp.segment_tick = (csp.segment_tick + 1) & 0x7fffffff;

    return current + step;
}

// 线程函数实现
void ERobMotorController::realtimeLoop() {
    // 设置实时优先级
    struct sched_param rt_param;
    rt_param.sched_priority = 80;
    sched_setscheduler(0, SCHED_FIFO, &rt_param);

    struct timespec ts, tleft;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    
    int64_t cycletime = cycle_time_us_ * 1000;  // 转换为纳秒
    
    // 初始化周期时间
    int ht = (ts.tv_nsec / 1000000) + 1;
    ts.tv_nsec = ht * 1000000;
    if (ts.tv_nsec >= NSEC_PER_SEC) {
        ts.tv_sec++;
        ts.tv_nsec -= NSEC_PER_SEC;
    }

    toff_ = 0;
    
    std::cout << "Real-time loop started with cycle time: " << cycle_time_us_ << " us" << std::endl;

    while (running_) {
        add_timespec(&ts, cycletime + toff_);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);

        // PDO数据交换
        ec_send_processdata();
        wkc_ = ec_receive_processdata(EC_TIMEOUTRET);

        if (in_op_ && wkc_ >= expected_wkc_) {
            std::lock_guard<std::mutex> lock(motor_data_mutex_);
            
            // 更新所有电机状态
            for (int i = 0; i < getMotorCount(); i++) {
                updateMotorStatus(i);
                
                // 准备发送数据
                motor_rxpdo_t rxpdo;
                rxpdo.mode_of_operation = static_cast<uint8_t>(motors_[i].current_mode);
                
                // 根据控制模式设置控制字和目标值
                switch (motors_[i].current_mode) {
                    case MotorControlMode::CSV:
                        rxpdo.controlword = generateControlWord(motors_[i].state, motors_[i].is_enabled);
                        rxpdo.target_velocity = motors_[i].target_velocity;
                        rxpdo.target_position = 0;
                        break;
                    case MotorControlMode::PP:
                        {
                            // PP模式需要特殊的控制字处理
                            static std::vector<bool> pp_setpoint_sent(getMotorCount(), false);
                            static std::vector<int> pp_control_sequence(getMotorCount(), 0);
                            
                            if (pp_control_sequence[i] == 0) {
                                // 第一步：发送基础使能
                                rxpdo.controlword = generateControlWord(motors_[i].state, motors_[i].is_enabled);
                                pp_control_sequence[i] = 1;
                            } else if (pp_control_sequence[i] == 1 && motors_[i].state == MotorState::OPERATION_ENABLED) {
                                // 第二步：发送新设定点标志
                                rxpdo.controlword = generatePPControlWord(motors_[i].state, motors_[i].is_enabled, true);
                                pp_setpoint_sent[i] = true;
                                pp_control_sequence[i] = 2;
                            } else if (pp_control_sequence[i] == 2) {
                                // 第三步：清除新设定点标志，保持使能
                                rxpdo.controlword = generatePPControlWord(motors_[i].state, motors_[i].is_enabled, false);
                                pp_control_sequence[i] = 3;
                            } else {
                                // 正常运行状态
                                rxpdo.controlword = generateControlWord(motors_[i].state, motors_[i].is_enabled);
                            }
                            
                            rxpdo.target_velocity = 0;
                            rxpdo.target_position = motors_[i].target_position;
                        }
                        break;
                    case MotorControlMode::CSP:
                        rxpdo.controlword = generateControlWord(motors_[i].state, motors_[i].is_enabled);
                        rxpdo.target_velocity = 0;
                        if (csp_params_[i].enabled) {
                            rxpdo.target_position = calculateCSPInterpolatedPosition(i);
                        } else {
                            rxpdo.target_position = motors_[i].actual_position;
                        }
                        break;
                }
                
                rxpdo.padding = 0;
                
                // 发送到EtherCAT从站
                int slave_id = i + 1;
                memcpy(ec_slave[slave_id].outputs, &rxpdo, sizeof(motor_rxpdo_t));
            }
            
            // 分布式时钟同步
            if (ec_slave[0].hasdc) {
                int64_t reftime = 0;
                ec_sync(reftime, cycletime, &toff_);
            }
        }
    }
    
    std::cout << "Real-time loop stopped" << std::endl;
}

void ERobMotorController::checkLoop() {
    std::cout << "Check loop started" << std::endl;
    
    int consecutive_errors = 0;
    const int MAX_CONSECUTIVE_ERRORS = 5;

    while (running_) {
        if (in_op_ && wkc_ < expected_wkc_) {
            consecutive_errors++;
            std::cout << "Communication error, WKC: " << wkc_ << " expected: " << expected_wkc_ << std::endl;
            
            if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
                std::cout << "Too many consecutive errors, checking slave states..." << std::endl;
                
                ec_readstate();
                for (int slave = 1; slave <= ec_slavecount; slave++) {
                    if (ec_slave[slave].state != EC_STATE_OPERATIONAL) {
                        std::cout << "Slave " << slave << " state: " << ec_slave[slave].state 
                                  << " AL status: " << ec_slave[slave].ALstatuscode << std::endl;
                    }
                }
                consecutive_errors = 0;
            }
        } else {
            consecutive_errors = 0;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    std::cout << "Check loop stopped" << std::endl;
}

// 已移除独立的CSP随动线程

// 时钟同步函数
void ERobMotorController::ec_sync(int64_t reftime, int64_t cycletime, int64_t* offsettime) {
    static int64_t integral = 0;
    int64_t delta;
    
    delta = (reftime) % cycletime;
    if (delta > (cycletime / 2)) {
        delta = delta - cycletime;
    }
    if (delta > 0) {
        integral++;
    }
    if (delta < 0) {
        integral--;
    }
    *offsettime = -(delta / 100) - (integral / 20);
    gl_delta_ = delta;
}

void ERobMotorController::add_timespec(struct timespec* ts, int64_t addtime) {
    int64_t sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if (ts->tv_nsec >= NSEC_PER_SEC) {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}

// 已移除复杂的视觉伺服运动规划相关函数