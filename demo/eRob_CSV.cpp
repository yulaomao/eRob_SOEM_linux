/*
 * 该程序为 EtherCAT 主站示例，负责初始化并配置 EtherCAT 从站，
 * 管理它们的状态，并处理实时数据交换。包含 PDO 映射配置、
 * 与分布式时钟（DC）同步的功能，以及伺服电机的多种运行模式控制。
 * 程序使用多线程实现实时处理和网络监控。
 */
//#include <QCoreApplication>


#include <stdio.h>
#include <string.h>
#include "ethercat.h"
#include <iostream>
#include <inttypes.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>

#include <sys/time.h>
#include <pthread.h>
#include <math.h>

#include <chrono>
#include <ctime>

#include <iostream>
#include <cstdint>

#include <sched.h>

// EtherCAT 通信全局变量
char IOmap[4096]; // EtherCAT 的 I/O 映射
int expectedWKC; // 期望的工作计数器（Work Counter）
boolean needlf; // 是否需要换行的标志
volatile int wkc; // 实时更新的工作计数器（多线程可见）
boolean inOP; // 系统是否处于 OP（运行）状态的标志
uint8 currentgroup = 0; // 当前通信组索引
int dorun = 0; // 线程是否运行的标志
bool start_ecatthread_thread; // 是否启动 EtherCAT 实时线程的标志
int ctime_thread; // EtherCAT 实时线程的周期（微秒）

int64 toff, gl_delta; // 时钟同步用的时间偏移和全局 delta

// EtherCAT 线程函数原型
OSAL_THREAD_FUNC ecatcheck(void *ptr); // 检查 EtherCAT 从站状态的线程函数原型
OSAL_THREAD_FUNC_RT ecatthread(void *ptr); // 实时 EtherCAT 处理线程函数原型

// EtherCAT 线程句柄
OSAL_THREAD_HANDLE thread1; // 检查线程的句柄
OSAL_THREAD_HANDLE thread2; // 实时线程的句柄

// 与 EtherCAT 分布式时钟（DC）同步的函数声明
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime);
// 向 timespec 结构中添加纳秒的工具函数声明
void add_timespec(struct timespec *ts, int64 addtime);

#define stack64k (64 * 1024) // 线程栈大小
#define NSEC_PER_SEC 1000000000   // 每秒纳秒数
#define EC_TIMEOUTMON 5000        // 监控超时（微秒）
#define MAX_VELOCITY 30000        // 最大速度
#define MAX_ACCELERATION 50000    // 最大加速度

// 伺服电机单位换算
float Cnt_to_deg = 0.000686645; // 计数到角度（度）的转换系数
int8_t SLAVE_ID; // EtherCAT 从站 ID

// RXPDO 结构（发送到从站的控制数据）
typedef struct {
    uint16_t controlword;      // 0x6040:0，16 位
    int32_t target_velocity;   // 0x60FF:0，32 位 (CSV模式使用)
    int32_t target_position;   // 0x607A:0，32 位 (PP模式使用)
    uint8_t mode_of_operation; // 0x6060:0，8 位
    uint8_t padding;           // 8 位对齐填充
} __attribute__((__packed__)) rxpdo_t;

// TXPDO 结构（从从站接收的状态数据）
typedef struct {
    uint16_t statusword;      // 0x6041:0，16 位
    int32_t actual_position;  // 0x6064:0，32 位
    int32_t actual_velocity;  // 0x606C:0，32 位
    int16_t actual_torque;    // 0x6077:0，16 位
} __attribute__((__packed__)) txpdo_t;

// 全局变量
volatile int target_position = 0;
pthread_mutex_t target_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t target_position_cond = PTHREAD_COND_INITIALIZER;
bool target_updated = false;
int32_t received_target = 0;

rxpdo_t rxpdo;  // 全局 RXPDO（发送给从站的数据）
txpdo_t txpdo;  // 全局 TXPDO（从从站接收的数据）

struct MotorStatus {
    bool is_operational;
    uint16_t status_word;
    int32_t actual_position;
    int32_t actual_velocity;
    int16_t actual_torque;
} motor_status;

// 更新电机状态信息的函数
void update_motor_status(int slave_id) {
    // 从 TXPDO 更新状态信息
    motor_status.status_word = txpdo.statusword;
    motor_status.actual_position = txpdo.actual_position;
    motor_status.actual_velocity = txpdo.actual_velocity;
    motor_status.actual_torque = txpdo.actual_torque;
    
    // 检查状态字以判断电机是否处于可操作状态
    // 状态字的位0-3 为 0111 表示已启用并就绪
    motor_status.is_operational = (txpdo.statusword & 0x0F) == 0x07;
}

//##################################################################################################
// 函数：为线程设置 CPU 亲和性
void set_thread_affinity(pthread_t thread, int cpu_core) {
    cpu_set_t cpuset; // CPU 集合，用于指定线程可运行的 CPU
    CPU_ZERO(&cpuset); // 清空 CPU 集合
    CPU_SET(cpu_core, &cpuset); // 将指定的 CPU 核加入集合

    // 设置线程的 CPU 亲和性
    int result = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (result != 0) {
    printf("无法为线程绑定 CPU %d\n", cpu_core); // 设置失败的错误信息
    } else {
    printf("线程已成功绑定到 CPU %d\n", cpu_core); // 设置成功的确认信息
    }
}

//##################################################################################################
// EtherCAT 测试函数原型
int erob_test();

uint16_t data_R;

int erob_test() {
    int rdl; // 读取数据长度
    SLAVE_ID = 1; // 将从站 ID 设为 1
    int i, j, oloop, iloop, chk; // 循环控制变量

    // 1. 调用 ec_config_init() 将从 INIT 切换到 PRE-OP 状态
    printf("__________STEP 1___________________\n");
    // 在指定的网络接口上初始化 EtherCAT 主站
    if (ec_init("enp3s0") <= 0) {
    printf("错误：无法初始化 EtherCAT 主站！\n");
    printf("请以 root 身份运行，检查以太网端口套接字连接。\n");
        printf("___________________________________________\n");
    return -1; // 初始化失败则返回错误
    }
    printf("EtherCAT master initialized successfully.\n");
    printf("___________________________________________\n");

    // 在网络上搜索 EtherCAT 从站
    if (ec_config_init(FALSE) <= 0) {
    printf("错误：未发现 EtherCAT 从站！\n");
        printf("___________________________________________\n");
    ec_close(); // 关闭 EtherCAT 连接
    return -1; // 没有发现从站则返回错误
    }
    printf("%d 个从站已发现并配置。\n", ec_slavecount); // 打印发现的从站数量
    printf("___________________________________________\n");

    // 2. 切换到 PRE-OP（预运行）状态以配置 PDO 寄存器
    printf("__________STEP 2___________________\n");

    // 检查从站是否准备好进行映射
    ec_readstate(); // 读取从站状态

    for(int i = 1; i <= ec_slavecount; i++) { // Loop through each slave
        if(ec_slave[i].state != EC_STATE_PRE_OP) { // 从站未处于 PRE-OP
            // 打印该从站的当前状态和 AL 状态码
            printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                   i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            printf("\n请求从站 %d 进入 INIT 状态\n", i); // 请求将状态改为 INIT
            ec_slave[i].state = EC_STATE_INIT; // 将从站状态设为 INIT
            printf("___________________________________________\n");
        } else { // If the slave is in PRE-OP state
            ec_slave[0].state = EC_STATE_PRE_OP; // 将主站（索引0）设为 PRE-OP
            /* 请求所有从站进入 EC_STATE_PRE_OP 状态 */
            ec_writestate(0); // Write the state change to the slave
            /* 等待所有从站进入 PRE-OP 状态 */
            if ((ec_statecheck(0, EC_STATE_PRE_OP,  3 * EC_TIMEOUTSTATE)) == EC_STATE_PRE_OP) {
                printf("State changed to EC_STATE_PRE_OP: %d \n", EC_STATE_PRE_OP);
                printf("___________________________________________\n");
            } else {
                printf("State EC_STATE_PRE_OP cannot be changed in step 2\n");
                return -1; // Return error if state change fails
            }
        }
    }

//##################################################################################################
    //3.- Map RXPOD
    printf("__________STEP 3___________________\n");

    // Modify PDO mapping configuration
    int retval = 0;
    uint16 map_1c12;
    uint8 zero_map = 0;
    uint32 map_object;
    uint16 clear_val = 0x0000;

    for(int i = 1; i <= ec_slavecount; i++) {
        // Clear RXPDO mapping
        retval += ec_SDOwrite(i, 0x1600, 0x00, FALSE, sizeof(zero_map), &zero_map, EC_TIMEOUTSAFE);
        
        // Control word (0x6040:0, 16 bits)
        map_object = 0x60400010;
        retval += ec_SDOwrite(i, 0x1600, 0x01, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Target velocity (0x60FF:0, 32 bits) - for CSV mode
        map_object = 0x60FF0020;
        retval += ec_SDOwrite(i, 0x1600, 0x02, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Target position (0x607A:0, 32 bits) - for PP mode
        map_object = 0x607A0020;
        retval += ec_SDOwrite(i, 0x1600, 0x03, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Operation mode (0x6060:0, 8 bits)
        map_object = 0x60600008;
        retval += ec_SDOwrite(i, 0x1600, 0x04, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Padding (8 bits padding)
        map_object = 0x00000008;
        retval += ec_SDOwrite(i, 0x1600, 0x05, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        uint8 map_count = 5;  // Now there are 5 objects, including padding
        retval += ec_SDOwrite(i, 0x1600, 0x00, FALSE, sizeof(map_count), &map_count, EC_TIMEOUTSAFE);
        
        // Configure RXPDO allocation
        clear_val = 0x0000;
        retval += ec_SDOwrite(i, 0x1c12, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);
        map_1c12 = 0x1600;
        retval += ec_SDOwrite(i, 0x1c12, 0x01, FALSE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
        map_1c12 = 0x0001;
        retval += ec_SDOwrite(i, 0x1c12, 0x00, FALSE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
    }

    printf("RXPDO mapping configuration result: %d\n", retval);
    if (retval < 0) {
        printf("RXPDO mapping failed\n");
        return -1;
    }

    printf("RXPDO 映射设置正确。\n");
    printf("___________________________________________\n");

    //........................................................................................
    // Map TXPOD
    retval = 0;
    uint16 map_1c13;
    for(int i = 1; i <= ec_slavecount; i++) {
        // Clear TXPDO mapping
        clear_val = 0x0000;
        retval += ec_SDOwrite(i, 0x1A00, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);

        // Status Word (0x6041:0, 16 bits)
        map_object = 0x60410010;
        retval += ec_SDOwrite(i, 0x1A00, 0x01, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        // Actual Position (0x6064:0, 32 bits)
        map_object = 0x60640020;
        retval += ec_SDOwrite(i, 0x1A00, 0x02, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        // Actual Velocity (0x606C:0, 32 bits)
        map_object = 0x606C0020;
        retval += ec_SDOwrite(i, 0x1A00, 0x03, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        // Actual Torque (0x6077:0, 16 bits)
        map_object = 0x60770010;
        retval += ec_SDOwrite(i, 0x1A00, 0x04, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);

        uint8 map_count = 4;  // Ensure mapping 4 objects
        retval += ec_SDOwrite(i, 0x1A00, 0x00, FALSE, sizeof(map_count), &map_count, EC_TIMEOUTSAFE);

        // Correctly configure TXPDO allocation
        clear_val = 0x0000;
        retval += ec_SDOwrite(i, 0x1C13, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);
        map_1c13 = 0x1A00;
        retval += ec_SDOwrite(i, 0x1C13, 0x01, FALSE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);
        map_1c13 = 0x0001;
        retval += ec_SDOwrite(i, 0x1C13, 0x00, FALSE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);


    }

    printf("Slave %d TXPDO mapping configuration result: %d\n", SLAVE_ID, retval);

    if (retval < 0) {
        printf("TXPDO Mapping failed\n");
        printf("___________________________________________\n");
        return -1;
    }

    printf("TXPDO 映射设置成功\n");
    printf("___________________________________________\n");

   //##################################################################################################

    //4.- Set ecx_context.manualstatechange = 1. Map PDOs for all slaves by calling ec_config_map().
   printf("__________STEP 4___________________\n");

   ecx_context.manualstatechange = 1; //Disable automatic state change
   osal_usleep(1e6); //Sleep for 1 second

    uint8 WA = 0; //Variable for write access
    uint8 my_RA = 0; //Variable for read access
    uint32 TIME_RA; //Variable for time read access

    // Print the information of the slaves found
    for (int i = 1; i <= ec_slavecount; i++) {
       // (void)ecx_FPWR(ecx_context.port, i, ECT_REG_DCSYNCACT, sizeof(WA), &WA, 5 * EC_TIMEOUTRET);
        printf("Name: %s\n", ec_slave[i].name); //Print the name of the slave
        printf("Slave %d: Type %d, Address 0x%02x, State Machine actual %d, required %d\n", 
               i, ec_slave[i].eep_id, ec_slave[i].configadr, ec_slave[i].state, EC_STATE_INIT);
        printf("___________________________________________\n");
        // Defer enabling Sync0 until after OP to avoid blocking transition
        ecx_dcsync0(&ecx_context, i, FALSE, 0, 0);
    }

    // Map the configured PDOs to the IOmap
    ec_config_map(&IOmap);

    printf("__________STEP 5___________________\n");

    // Ensure all slaves are in PRE-OP state
    ec_readstate();
    for(int i = 1; i <= ec_slavecount; i++) {
        if(ec_slave[i].state != EC_STATE_PRE_OP) {
            printf("Slave %d not in PRE-OP state. Current state: %d, StatusCode=0x%4.4x : %s\n", 
                   i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            return -1;
        }
    }

    // Configure distributed clock
    printf("正在配置 DC...\n");
    ec_configdc();
    osal_usleep(200000);  // Wait for DC configuration to take effect

    // Request to switch to SAFE-OP state before confirming DC configuration
    for(int i = 1; i <= ec_slavecount; i++) {
        printf("Slave %d DC status: 0x%4.4x\n", i, ec_slave[i].DCactive);
        if(ec_slave[i].hasdc && !ec_slave[i].DCactive) {
            printf("DC not active for slave %d\n", i);
        }
    }

    // 请求切换到 SAFE-OP 状态
    printf("请求进入 SAFE_OP 状态...\n");
    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);
    osal_usleep(200000);  // Give enough time for state transition

    // Check the result of the state transition
    chk = 40;
    do {
        ec_readstate();
        for(int i = 1; i <= ec_slavecount; i++) {
            if(ec_slave[i].state != EC_STATE_SAFE_OP) {
                printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                       i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            }
        }
        osal_usleep(100000);
    } while (chk-- && (ec_slave[0].state != EC_STATE_SAFE_OP));

    if (ec_slave[0].state != EC_STATE_SAFE_OP) {
        printf("Failed to reach SAFE_OP state\n");
        return -1;
    }

    printf("Successfully reached SAFE_OP state\n");

    // Calculate the expected Work Counter (WKC)
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC; // Calculate expected WKC based on outputs and inputs
    printf("Calculated workcounter %d\n", expectedWKC);

    // Read and display basic status information of the slaves
    ec_readstate(); // Read the state of all slaves
    for(int i = 1; i <= ec_slavecount; i++) {
        printf("Slave %d\n", i);
        printf("  State: %02x\n", ec_slave[i].state); // Print the state of the slave
        printf("  ALStatusCode: %04x\n", ec_slave[i].ALstatuscode); // Print the AL status code
        printf("  Delay: %d\n", ec_slave[i].pdelay); // Print the delay of the slave
        printf("  Has DC: %d\n", ec_slave[i].hasdc); // Check if the slave supports Distributed Clock
        printf("  DC Active: %d\n", ec_slave[i].DCactive); // Check if DC is active for the slave
        printf("  DC supported: %d\n", ec_slave[i].hasdc); // Print if DC is supported
    }

    // Read DC synchronization configuration using the correct parameters
    for(int i = 1; i <= ec_slavecount; i++) {
        uint16_t dcControl = 0; // Variable to hold DC control configuration
        int32_t cycleTime = 0; // Variable to hold cycle time
        int32_t shiftTime = 0; // Variable to hold shift time
        int size; // Variable to hold size for reading

        // Read DC synchronization configuration, adding the correct size parameter
        size = sizeof(dcControl);
        if (ec_SDOread(i, 0x1C32, 0x01, FALSE, &size, &dcControl, EC_TIMEOUTSAFE) > 0) {
            printf("Slave %d DC Configuration:\n", i);
            printf("  DC Control: 0x%04x\n", dcControl); // Print the DC control configuration
            
            size = sizeof(cycleTime);
            if (ec_SDOread(i, 0x1C32, 0x02, FALSE, &size, &cycleTime, EC_TIMEOUTSAFE) > 0) {
                printf("  Cycle Time: %d ns\n", cycleTime); // Print the cycle time
            }

        }
    }

    printf("__________STEP 6___________________\n");

    // While still in SAFE_OP, preconfigure operation mode via SDO
    {
        uint8 operation_mode_csv = 9; // CSV
        for (int i = 1; i <= ec_slavecount; i++) {
            int ret = ec_SDOwrite(i, 0x6060, 0x00, FALSE, sizeof(operation_mode_csv), &operation_mode_csv, EC_TIMEOUTSAFE);
            if (ret <= 0) {
                printf("警告：在 SAFE_OP 中为从站 %d 预设置 0x6060 (CSV) 失败\n", i);
            }
        }
    }

    // Start the EtherCAT thread for real-time processing
    start_ecatthread_thread = TRUE; // Flag to indicate that the EtherCAT thread should start
    osal_thread_create_rt(&thread1, stack64k * 2, (void *)&ecatthread, (void *)&ctime_thread); // Create the real-time EtherCAT thread
    // set_thread_affinity(*thread1, 4); // Optional: Set CPU affinity for the thread
    osal_thread_create(&thread2, stack64k * 2, (void *)&ecatcheck, NULL); // Create the EtherCAT check thread
    // set_thread_affinity(*thread2, 5); // Optional: Set CPU affinity for the thread
    printf("___________________________________________\n");

    my_RA = 0; // Reset read access variable


    // 8. Transition to OP state
    printf("__________STEP 8___________________\n");

    // 修改：在转换到OP状态之前，确保所有从站都正确配置
    for (int i = 1; i <= ec_slavecount; i++) {
        // 检查从站是否在SAFE_OP状态
        if (ec_slave[i].state != EC_STATE_SAFE_OP) {
            printf("警告：从站 %d 在尝试切换到 OP 之前未处于 SAFE_OP 状态\n", i);
            return -1;
        }
    }

    // 初始化PDO数据
    rxpdo.controlword = 0x0000;  // 先发送停止命令
    rxpdo.target_velocity = 0;
    rxpdo.mode_of_operation = 9;  // CSV mode
    rxpdo.padding = 0;

    // 发送初始PDO数据
    for (int slave = 1; slave <= ec_slavecount; slave++) {
        memcpy(ec_slave[slave].outputs, &rxpdo, sizeof(rxpdo_t));
    }

    // 发送几次数据以确保从站准备好
    for (int i = 0; i < 5; i++) {
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        printf("Pre-OP data exchange %d: wkc=%d, expected=%d\n", i+1, wkc, expectedWKC);
        osal_usleep(10000);  // 10ms delay
    }

    // 现在请求切换到 OP（运行）状态
    printf("请求进入 OPERATIONAL 状态...\n");
    // Ensure Sync0 is enabled before transitioning to OP for DC slaves
    for (int i = 1; i <= ec_slavecount; i++) {
        if (ec_slave[i].hasdc) {
            ecx_dcsync0(&ecx_context, i, TRUE, 1000000, 0);
        }
    }
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);

    // 等待状态转换，同时持续进行PDO交换（部分驱动需要）
    int timeout_count = 0;
    const int MAX_TIMEOUT = 2500;  // ~5秒超时（2ms周期）
    while (timeout_count < MAX_TIMEOUT) {
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        // 小步轮询状态，减少打印频率
        ec_readstate();
        bool all_operational = true;
        for (int i = 1; i <= ec_slavecount; i++) {
            if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                all_operational = false;
                if ((timeout_count % 50) == 0) {
                    printf("Slave %d state: 0x%02x, AL status: 0x%04x\n",
                           i, ec_slave[i].state, ec_slave[i].ALstatuscode);
                }
                // Try to clear SAFE_OP+ERROR and push to OP individually
                if (ec_slave[i].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                    ec_slave[i].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                    ec_writestate(i);
                } else if (ec_slave[i].state == EC_STATE_SAFE_OP) {
                    ec_slave[i].state = EC_STATE_OPERATIONAL;
                    ec_writestate(i);
                }
            }
        }
        if (all_operational) {
            printf("All slaves reached OPERATIONAL state\n");
            break;
        }
        if ((timeout_count % 100) == 0) {
            // periodically nudge state request to OP
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            ec_writestate(0);
        }
        timeout_count++;
        osal_usleep(1000);  // 1ms周期，与DC周期一致
    }

    if (timeout_count >= MAX_TIMEOUT) {
        printf("ERROR: Failed to reach OPERATIONAL state within timeout\n");
        for (int i = 1; i <= ec_slavecount; i++) {
            printf("Slave %d: State=0x%02x, ALstatus=0x%04x : %s\n",
                   i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                   ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
        }
        return -1;
    }

    // 标记OP，允许实时线程开始循环
    inOP = TRUE;

    // Enable Sync0 only after OP is reached
    for (int i = 1; i <= ec_slavecount; i++) {
        if (ec_slave[i].hasdc) {
            ecx_dcsync0(&ecx_context, i, TRUE, 1000000, 0);
        }
    }

    printf("State changed to EC_STATE_OPERATIONAL: %d\n", EC_STATE_OPERATIONAL);
    printf("___________________________________________\n");

    // Read and display the state of all slaves
    ec_readstate(); // Read the state of all slaves
    for (int i = 1; i <= ec_slavecount; i++) {
        printf("Slave %d: Type %d, Address 0x%02x, State Machine actual %d, required %d\n", 
               i, ec_slave[i].eep_id, ec_slave[i].configadr, ec_slave[i].state, EC_STATE_OPERATIONAL); // Print slave information
        printf("Name: %s\n", ec_slave[i].name); // Print the name of the slave
        printf("___________________________________________\n");
    }

    // 9. Configure servomotor and mode operation
    printf("__________STEP 9___________________\n");

    if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
    printf("所有从站已进入运行（OP）状态。\n");
        
        // 配置每个从站的参数
        for (int i = 1; i <= ec_slavecount; i++) {
            uint8 operation_mode = 9;  // CSV mode
            uint16_t Control_Word = 0;
            int32_t Max_Velocity = 5000;
            int32_t Max_Acceleration = 5000;
            int32_t Quick_Stop_Decel = 10000;
            int32_t Profile_Decel = 5000;
            
            printf("Configuring slave %d...\n", i);
            
            // 设置操作模式
            if (ec_SDOwrite(i, 0x6060, 0x00, FALSE, sizeof(operation_mode), &operation_mode, EC_TIMEOUTSAFE) <= 0) {
                printf("Failed to set operation mode for slave %d\n", i);
            }
            
            // 设置速度参数
            ec_SDOwrite(i, 0x6080, 0x00, FALSE, sizeof(Max_Velocity), &Max_Velocity, EC_TIMEOUTSAFE);
            ec_SDOwrite(i, 0x60C5, 0x00, FALSE, sizeof(Max_Acceleration), &Max_Acceleration, EC_TIMEOUTSAFE);
            ec_SDOwrite(i, 0x6085, 0x00, FALSE, sizeof(Quick_Stop_Decel), &Quick_Stop_Decel, EC_TIMEOUTSAFE);
            ec_SDOwrite(i, 0x6084, 0x00, FALSE, sizeof(Profile_Decel), &Profile_Decel, EC_TIMEOUTSAFE);
            
            // 验证模式设置
            uint8 actual_mode;
            int size = sizeof(actual_mode);
            if (ec_SDOread(i, 0x6061, 0x00, FALSE, &size, &actual_mode, EC_TIMEOUTSAFE) > 0) {
                printf("Slave %d actual operation mode: %d\n", i, actual_mode);
            }
        }

        // 等待实时线程稳定运行
        osal_usleep(2000000);  // 2秒
        
        printf("System running in operational mode...\n");
        while(1) {
            osal_usleep(1000000);  // 1秒检查一次
            
            // 检查通信状态
            if (wkc < expectedWKC) {
                printf("WARNING: Communication issue detected (wkc=%d, expected=%d)\n", wkc, expectedWKC);
            }
        }
    }

    osal_usleep(1e6);

    ec_close();

    printf("\n请求所有从站进入 INIT 状态\n");
     ec_slave[0].state = EC_STATE_INIT;
     /* request INIT state for all slaves */
     ec_writestate(0);

    printf("EtherCAT master closed.\n");

    return 0;
}

/*
 * PI 控制，用于将 Linux 时间与分布式时钟（DC）时间同步。
 * 计算需要应用到本地时钟的偏移量以对齐 DC 时间。
 */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime) {
    static int64 integral = 0; // PI 控制中的积分项
    int64 delta; // 记录参考时间与周期时间之差
    delta = (reftime) % cycletime; // 计算差值
    if (delta > (cycletime / 2)) {
        delta = delta - cycletime; // 若差值大于半个周期则调整
    }
    if (delta > 0) {
        integral++; // 若差值为正则增加积分
    }
    if (delta < 0) {
        integral--; // 若差值为负则减少积分
    }
    *offsettime = -(delta / 100) - (integral / 20); // 计算并输出偏移量
    gl_delta = delta; // 更新全局 delta
}

/*
 * 向 timespec 结构添加指定的纳秒数。
 * 该函数会处理跨秒的进位。
 */
void add_timespec(struct timespec *ts, int64 addtime) {
    int64 sec, nsec; // 秒和纳秒部分

    nsec = addtime % NSEC_PER_SEC; // 计算要添加的纳秒
    sec = (addtime - nsec) / NSEC_PER_SEC; // 计算要添加的秒数
    ts->tv_sec += sec; // 增加秒
    ts->tv_nsec += nsec; // 增加纳秒
    if (ts->tv_nsec >= NSEC_PER_SEC) { // 若纳秒超出一秒则进位
        nsec = ts->tv_nsec % NSEC_PER_SEC; // 计算剩余纳秒
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC; // 增加秒数
        ts->tv_nsec = nsec; // 设置剩余纳秒
    }
}

/* 
 * EtherCAT check thread function
 * This function monitors the state of the EtherCAT slaves and attempts to recover 
 * any slaves that are not in the operational state.
 */
OSAL_THREAD_FUNC ecatcheck(void *ptr) {
    int slave; // Variable to hold the current slave index
    (void)ptr; // Not used
    int consecutive_errors = 0;
    const int MAX_CONSECUTIVE_ERRORS = 5;

    while (1) {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)) {
            if (needlf) {
                needlf = FALSE;
                printf("\n");
            }
            
            // Increase the consecutive error count
            if (wkc < expectedWKC) {
                consecutive_errors++;
                printf("警告：工作计数器错误 (%d/%d)，连续错误次数：%d\n", 
                       wkc, expectedWKC, consecutive_errors);
            } else {
                consecutive_errors = 0;
            }

            // If the consecutive errors exceed the threshold, attempt reinitialization
            if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
                printf("错误：连续错误次数过多，尝试恢复中...\n");
                ec_group[currentgroup].docheckstate = TRUE;
                // Reset the error count
                consecutive_errors = 0;
            }

            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++) {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                        printf("错误：从站 %d 处于 SAFE_OP + ERROR，尝试 ACK。\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
                        printf("警告：从站 %d 处于 SAFE_OP，尝试切换到 OPERATIONAL。\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    } else if (ec_slave[slave].state > EC_STATE_NONE) {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
                            ec_slave[slave].islost = FALSE;
                            printf("消息：从站 %d 已重新配置\n", slave);
                        }
                    } else if (!ec_slave[slave].islost) {
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state) {
                            ec_slave[slave].islost = TRUE;
                            printf("错误：从站 %d 丢失\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost) {
                    if (!ec_slave[slave].state) {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
                            ec_slave[slave].islost = FALSE;
                            printf("消息：从站 %d 已恢复\n", slave);
                        }
                    } else {
                        ec_slave[slave].islost = FALSE;
                        printf("消息：发现从站 %d\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate) {
                printf("OK：所有从站已恢复到运行（OP）状态。\n");
            }
        }
        osal_usleep(10000); 
    }
}

/* 
 * RT EtherCAT thread function
 * This function handles the real-time processing of EtherCAT data. 
 * It sends and receives process data in a loop, synchronizing with the 
 * distributed clock if available, and ensuring timely execution based on 
 * the specified cycle time.
 */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr) {
    struct timespec ts, tleft;
    int ht;
    int64 cycletime;
    int missed_cycles = 0;
    const int MAX_MISSED_CYCLES = 10;
    struct timespec cycle_start, cycle_end;
    long cycle_time_ns;

    // Set thread to realtime, but below main priority to avoid starvation
    struct sched_param rt_param;
    rt_param.sched_priority = 80;
    sched_setscheduler(0, SCHED_FIFO, &rt_param);

    clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1;
    ts.tv_nsec = ht * 1000000;
    if (ts.tv_nsec >= NSEC_PER_SEC) {
        ts.tv_sec++;
        ts.tv_nsec -= NSEC_PER_SEC;
    }
    cycletime = *(int *)ptr * 1000;

    toff = 0;
    dorun = 0;

    // Initialize PDO (also valid before OP)
    rxpdo.controlword = 0x0080;
    rxpdo.target_velocity = 0;
    rxpdo.target_position = 0;
    rxpdo.mode_of_operation = 9;  // CSV mode (9)
    rxpdo.padding = 0;

    for (int slave = 1; slave <= ec_slavecount; slave++) {
        memcpy(ec_slave[slave].outputs, &rxpdo, sizeof(rxpdo_t));
    }
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    int step = 0;
    int retry_count = 0;
    const int MAX_RETRY = 3;
    bool mode_switched = false;  // 标记是否已切换到PP模式
    int csv_run_cycles = 0;      // CSV模式运行的周期数
    const int CSV_RUN_TIME_CYCLES = 2000;  // 2秒 = 2000个1ms周期

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &cycle_start);
        
        add_timespec(&ts, cycletime + toff);
        if (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft) != 0) {
            missed_cycles++;
            if (missed_cycles >= MAX_MISSED_CYCLES) {
                clock_gettime(CLOCK_MONOTONIC, &ts);
                ts.tv_nsec = ((ts.tv_nsec / 1000000) + 1) * 1000000;
                if (ts.tv_nsec >= NSEC_PER_SEC) {
                    ts.tv_sec++;
                    ts.tv_nsec -= NSEC_PER_SEC;
                }
                missed_cycles = 0;
            }
        } else {
            missed_cycles = 0;
        }
        
        dorun++;

        // Always exchange PDOs to support OP transition with DC
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        if (inOP) {

            if (wkc >= expectedWKC) {
                retry_count = 0;  // Reset retry counter
                
                for (int slave = 1; slave <= ec_slavecount; slave++) {
                    memcpy(&txpdo, ec_slave[slave].inputs, sizeof(txpdo_t));
                }

                // State machine control (EN enable sequence then run)
                if (step <= 1500) {
                    // 初始状态
                    rxpdo.controlword = 0x0080;
                    rxpdo.target_velocity = 0;
                    rxpdo.target_position = 0;
                    rxpdo.mode_of_operation = 9;  // CSV mode
                } else if (step <= 1800) {
                    // Shutdown命令
                    rxpdo.controlword = 0x0006;
                    rxpdo.target_velocity = 0;
                    rxpdo.target_position = 0;
                    rxpdo.mode_of_operation = 9;
                } else if (step <= 2000) {
                    // Switch On命令
                    rxpdo.controlword = 0x0007;
                    rxpdo.target_velocity = 0;
                    rxpdo.target_position = 0;
                    rxpdo.mode_of_operation = 9;
                } else if (step <= 2400) {
                    // Enable Operation命令
                    rxpdo.controlword = 0x000F;
                    rxpdo.target_velocity = 0;
                    rxpdo.target_position = 0;
                    rxpdo.mode_of_operation = 9;
                } else {
                    // 运行阶段
                    if (!mode_switched && csv_run_cycles < CSV_RUN_TIME_CYCLES) {
                        // CSV模式运行阶段（2秒）
                        rxpdo.controlword = 0x000F;
                        rxpdo.target_velocity = 2000;  // CSV模式速度
                        rxpdo.target_position = 0;     // CSV模式不使用位置
                        rxpdo.mode_of_operation = 9;   // CSV mode
                        csv_run_cycles++;
                        
                        if (dorun % 200 == 0) {
                            printf("CSV模式运行中: cycle=%d/%d, vel=%d\n", 
                                   csv_run_cycles, CSV_RUN_TIME_CYCLES, rxpdo.target_velocity);
                        }
                    } else if (!mode_switched) {
                        // 切换到PP模式 - 保持使能状态下切换
                        printf(">>> 切换到PP模式，目标位置：0\n");
                        
                        // 通过SDO设置PP模式参数
                        for (int i = 1; i <= ec_slavecount; i++) {
                            uint8 pp_mode = 1;
                            uint32_t profile_velocity = 5000;    // 增加profile速度
                            uint32_t profile_acceleration = 10000;
                            uint32_t profile_deceleration = 10000;
                            
                            // 先设置PP模式参数
                            ec_SDOwrite(i, 0x6081, 0x00, FALSE, sizeof(profile_velocity), &profile_velocity, EC_TIMEOUTSAFE);
                            ec_SDOwrite(i, 0x6083, 0x00, FALSE, sizeof(profile_acceleration), &profile_acceleration, EC_TIMEOUTSAFE);
                            ec_SDOwrite(i, 0x6084, 0x00, FALSE, sizeof(profile_deceleration), &profile_deceleration, EC_TIMEOUTSAFE);
                            
                            // 设置目标位置
                            int32_t target_pos = 0;
                            ec_SDOwrite(i, 0x607A, 0x00, FALSE, sizeof(target_pos), &target_pos, EC_TIMEOUTSAFE);
                            
                            // 最后切换模式
                            ec_SDOwrite(i, 0x6060, 0x00, FALSE, sizeof(pp_mode), &pp_mode, EC_TIMEOUTSAFE);
                            
                            printf("PP模式参数设置完成: vel=%d, acc=%d, dec=%d, target=%d\n", 
                                   profile_velocity, profile_acceleration, profile_deceleration, target_pos);
                        }
                        
                        // 保持使能状态，切换模式
                        rxpdo.controlword = 0x000F;  // 保持Enable Operation状态
                        rxpdo.target_velocity = 0;    // CSV模式不再使用
                        rxpdo.target_position = 0;    // PP模式目标位置
                        rxpdo.mode_of_operation = 1;  // 切换到PP mode
                        mode_switched = true;
                        
                        printf(">>> PP模式切换完成，保持使能状态\n");
                    } else {
                        // PP模式正常运行 - 使用更明确的运动触发序列
                        static bool setpoint_sent = false;
                        static int pp_step = 0;
                        
                        if (pp_step < 50) {
                            // 步骤1: 确保在PP模式下稳定运行
                            rxpdo.controlword = 0x000F;  // Enable Operation
                            rxpdo.target_velocity = 0;
                            rxpdo.target_position = 0;   // 目标位置为0
                            rxpdo.mode_of_operation = 1; // PP mode
                            pp_step++;
                            
                            if (pp_step == 1) {
                                printf(">>> PP模式稳定期，等待模式切换完成\n");
                            }
                        } else if (pp_step < 100) {
                            // 步骤2: 发送新的目标位置
                            rxpdo.controlword = 0x001F;  // Enable Operation + new setpoint + halt=0
                            rxpdo.target_velocity = 0;
                            rxpdo.target_position = 0;   // 目标位置为0
                            rxpdo.mode_of_operation = 1; // PP mode
                            pp_step++;
                            
                            if (pp_step == 51) {
                                printf(">>> PP模式发送新目标位置：0，当前位置：%d\n", txpdo.actual_position);
                            }
                        } else {
                            // 步骤3: 保持运行状态
                            rxpdo.controlword = 0x000F;  // 保持Enable Operation状态
                            rxpdo.target_velocity = 0;
                            rxpdo.target_position = 0;   // 目标位置为0
                            rxpdo.mode_of_operation = 1; // PP mode
                        }
                        
                        // 检查是否到达目标位置
                        if (dorun % 500 == 0) {
                            int position_error = abs(txpdo.actual_position - 0);
                            printf("PP模式运行中: pos=%d, target=0, error=%d, vel=%d, status=0x%04x, step=%d\n", 
                                   txpdo.actual_position, position_error, txpdo.actual_velocity, txpdo.statusword, pp_step);
                            
                            if (position_error < 100 && abs(txpdo.actual_velocity) < 50) {
                                printf(">>> 已到达目标位置0附近\n");
                            }
                        }
                    }
                }

                // Send data to slaves
                for (int slave = 1; slave <= ec_slavecount; slave++) {
                    memcpy(ec_slave[slave].outputs, &rxpdo, sizeof(rxpdo_t));
                }

                if (dorun % 200 == 0 && step > 2400) {
                    const char* mode_name = (rxpdo.mode_of_operation == 9) ? "CSV" : "PP";
                    
                    // 添加状态字解析
                    const char* state_desc = "";
                    uint16_t sw = txpdo.statusword;
                    if ((sw & 0x004F) == 0x0000) state_desc = "Not ready to switch on";
                    else if ((sw & 0x004F) == 0x0040) state_desc = "Switch on disabled";
                    else if ((sw & 0x006F) == 0x0021) state_desc = "Ready to switch on";
                    else if ((sw & 0x006F) == 0x0023) state_desc = "Switched on";
                    else if ((sw & 0x006F) == 0x0027) state_desc = "Operation enabled";
                    else if ((sw & 0x006F) == 0x0007) state_desc = "Quick stop active";
                    else if ((sw & 0x004F) == 0x000F) state_desc = "Fault reaction active";
                    else if ((sw & 0x004F) == 0x0008) state_desc = "Fault";
                    else state_desc = "Unknown state";
                    
                    printf("Status [%s]: SW=0x%04x (%s), pos=%d, vel=%d, target_vel=%d, target_pos=%d\n",
                           mode_name, txpdo.statusword, state_desc,
                           txpdo.actual_position, txpdo.actual_velocity,
                           rxpdo.target_velocity, rxpdo.target_position);
                }

                if (step < 8000) {
                    step++;
                }
            } else {
                retry_count++;
                if (retry_count >= MAX_RETRY) {
                    printf("错误：通信失败，重试 %d 次后仍未成功\n", retry_count);
                    retry_count = 0;
                }
            }

            // clock synchronization
            if (ec_slave[0].hasdc) {
                ec_sync(ec_DCtime, cycletime, &toff);
            }
        }

        // monitor cycle time
        clock_gettime(CLOCK_MONOTONIC, &cycle_end);
        cycle_time_ns = (cycle_end.tv_sec - cycle_start.tv_sec) * NSEC_PER_SEC +
                       (cycle_end.tv_nsec - cycle_start.tv_nsec);
        
        if (cycle_time_ns > cycletime * 2 && dorun % 1000 == 0) {
            printf("警告：周期时间超限：%ld ns（期望：%ld ns）\n",
                   cycle_time_ns, cycletime);
        }
    }
}

int correct_count = 0;
int incorrect_count = 0;
int test_count_sum = 100;
int test_count = 0;
float correct_rate = 0;

// Main function
int main(int argc, char **argv) {
    needlf = FALSE;
    inOP = FALSE;
    start_ecatthread_thread = FALSE;
    dorun = 0;

    ctime_thread = 1000;  // Communication period

    // Set the highest real-time priority
    struct sched_param param;
    param.sched_priority = 99;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }

    // Lock memory
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
    }

    // Set CPU affinity to two cores
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset);  // Use CPU core 2
    CPU_SET(3, &cpuset);  // Use CPU core 3

    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) {
        perror("sched_setaffinity");
        return EXIT_FAILURE;
    }

    printf("Running on CPU cores 2 and 3\n");
    erob_test();
    printf("End program\n");

    return EXIT_SUCCESS;
}
