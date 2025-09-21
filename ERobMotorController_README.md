# ERobMotorController

基于SOEM (Simple Open EtherCAT Master)的电机控制器类，提供简单易用的API来控制EtherCAT伺服电机。

## 功能特性

### 基础功能
- 自动扫描并初始化所有EtherCAT电机
- 获取电机数量、位置、速度、转矩等状态信息
- 电机使能/禁能控制
- 支持多种控制模式：CSV、CSP、PP

### 控制模式
1. **PP (Profile Position)**: 单步位置控制，指定目标位置和速度
2. **CSV (Cyclic Synchronous Velocity)**: 速度控制模式
3. **CSP (Cyclic Synchronous Position)**: 位置随动控制，支持实时轨迹跟踪

### CSP随动控制特性
- 每16ms接收目标位置更新
- 每1ms进行轨迹插值计算
- 自动根据当前位置和速度进行轨迹规划
- 支持最大速度和加速度限制

## 编译和安装

### 依赖要求
- Linux系统 (Ubuntu 18.04+ 推荐)
- CMake 3.9+
- GCC/G++ 编译器
- EtherCAT网络接口卡
- Root权限 (用于EtherCAT网络访问)

### 编译步骤
```bash
# 创建build目录
mkdir build
cd build

# 配置和编译
cmake ..
make

# 可选：安装到系统
sudo make install
```

### 编译产物
- `libERobMotorController.so`: 电机控制器动态库
- `motor_controller_example`: 使用示例程序

## 使用方法

### 基本使用流程

```cpp
#include "ERobMotorController.h"

int main() {
    // 1. 创建控制器实例
    ERobMotorController controller("enp3s0", 1000);  // 网络接口, 1ms周期
    
    // 2. 初始化
    if (!controller.initialize()) {
        std::cerr << "初始化失败!" << std::endl;
        return -1;
    }
    
    // 3. 获取电机数量
    int motor_count = controller.getMotorCount();
    std::cout << "发现 " << motor_count << " 个电机" << std::endl;
    
    // 4. 使能电机
    controller.enableAllMotors(true);
    
    // 5. 控制电机
    // PP模式 - 单步位置控制
    controller.moveToPosition(0, 10000, 1000, true);  // 电机0移动到位置10000
    
    // CSP模式 - 随动控制
    controller.enableCSPFollow(0, true);
    controller.setCSPTargetPosition(0, 15000);  // 设置目标位置
    
    // 6. 读取状态
    int32_t position = controller.getMotorPosition(0);
    int32_t velocity = controller.getMotorVelocity(0);
    
    // 7. 清理
    controller.enableAllMotors(false);
    // 析构函数会自动调用shutdown()
    
    return 0;
}
```

### API参考

#### 初始化和基础控制
```cpp
// 构造函数
ERobMotorController(const std::string& interface_name, int cycle_time_us = 1000);

// 初始化EtherCAT网络
bool initialize();

// 关闭连接
void shutdown();

// 获取电机数量
int getMotorCount() const;

// 使能/禁能电机
bool enableMotor(int motor_id, bool enable);
bool enableAllMotors(bool enable);
```

#### 状态读取
```cpp
// 获取电机位置 (编码器计数)
int32_t getMotorPosition(int motor_id) const;

// 获取电机速度 (计数/秒)
int32_t getMotorVelocity(int motor_id) const;

// 获取电机转矩
int16_t getMotorTorque(int motor_id) const;

// 获取电机状态
MotorState getMotorState(int motor_id) const;
```

#### 位置控制 (PP模式)
```cpp
// 移动到指定位置
bool moveToPosition(int motor_id, int32_t target_position, int32_t velocity, bool blocking = false);

// 检查是否到达目标位置
bool isMotorAtTarget(int motor_id, int32_t tolerance = 100) const;
```

#### CSP随动控制
```cpp
// 启用/禁用CSP随动模式
bool enableCSPFollow(int motor_id, bool enable);

// 设置目标位置
bool setCSPTargetPosition(int motor_id, int32_t target_position);

// 设置运动参数
bool setCSPMotionParams(int motor_id, int32_t max_velocity, int32_t max_acceleration);
```

#### 工具函数
```cpp
// 编码器计数与角度转换
static double countsToAngle(int32_t counts);    // 计数 -> 角度(度)
static int32_t angleToCounts(double angle);     // 角度(度) -> 计数
```

## 运行示例

### 运行权限
EtherCAT通信需要root权限：
```bash
sudo ./motor_controller_example [网络接口名]
```

### 网络接口配置
查找EtherCAT网络接口：
```bash
ip link show
# 或
ifconfig -a
```

常见接口名：
- `enp3s0`
- `eth0`
- `ens33`

### 示例程序功能
1. **状态读取**: 显示所有电机的位置、速度、转矩状态
2. **电机使能**: 启用所有电机
3. **PP位置控制**: 单个电机移动90度
4. **CSP随动控制**: 连续轨迹跟踪
5. **多电机协调**: 多个电机同时运动

## 技术细节

### 线程架构
- **实时线程**: 1ms周期，处理EtherCAT数据交换和轨迹插值
- **检查线程**: 10ms周期，监控通信状态和从站状态
- **CSP随动线程**: 16ms周期，接收外部目标位置更新

### PDO映射
- **RXPDO** (发送到电机):
  - 控制字 (16位)
  - 目标速度 (32位)
  - 目标位置 (32位)
  - 操作模式 (8位)
  - 填充 (8位)

- **TXPDO** (从电机接收):
  - 状态字 (16位)
  - 实际位置 (32位)
  - 实际速度 (32位)
  - 实际转矩 (16位)

### CSP轨迹插值算法
1. 计算到目标位置的距离
2. 根据当前速度计算停止距离
3. 决定加速、匀速或减速阶段
4. 计算下一个1ms周期的目标位置
5. 确保不超过最大速度和加速度限制

## 故障排除

### 常见问题
1. **初始化失败**
   - 检查网络接口名是否正确
   - 确保以root权限运行
   - 检查EtherCAT电缆连接

2. **通信错误**
   - 检查Work Counter (WKC)
   - 查看从站状态
   - 检查网络接口配置

3. **电机不响应**
   - 检查电机使能状态
   - 确认操作模式设置
   - 查看电机状态字

### 调试信息
程序会输出详细的调试信息，包括：
- EtherCAT初始化过程
- 从站发现和配置
- 状态转换过程
- 实时通信状态

## 许可证

本项目基于SOEM库开发，遵循GPLv2许可证。

## 贡献

欢迎提交问题报告和改进建议！