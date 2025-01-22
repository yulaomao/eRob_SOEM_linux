/* 
 * This program is an EtherCAT master implementation that initializes and configures EtherCAT slaves,
 * manages their states, and handles real-time data exchange. It includes functions for setting up 
 * PDO mappings, synchronizing time with the distributed clock, and controlling servomotors in 
 * various operational modes. The program also features multi-threading for real-time processing 
 * and monitoring of the EtherCAT network.
 */
//#include <QCoreApplication>


#include <stdio.h>
#include <string.h>
#include "ethercat.h"
#include <iostream>
#include <inttypes.h>
#include <time.h>
#include <unistd.h>

#include <sys/time.h>
#include <pthread.h>
#include <math.h>

#include <chrono>
#include <ctime>

#include <iostream>
#include <cstdint>

#include <sched.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

// Global variables for EtherCAT communication
char IOmap[4096]; // I/O mapping for EtherCAT
int expectedWKC; // Expected Work Counter
boolean needlf; // Flag to indicate if a line feed is needed
volatile int wkc; // Work Counter (volatile to ensure it is updated correctly in multi-threaded context)
boolean inOP; // Flag to indicate if the system is in operational state
uint8 currentgroup = 0; // Current group for EtherCAT communication
int dorun = 0; // Flag to indicate if the thread should run
bool start_ecatthread_thread; // Flag to start the EtherCAT thread
int ctime_thread; // Cycle time for the EtherCAT thread

int64 toff, gl_delta; // Time offset and global delta for synchronization

// Function prototypes for EtherCAT thread functions
OSAL_THREAD_FUNC ecatcheck(void *ptr); // Function to check the state of EtherCAT slaves
OSAL_THREAD_FUNC ecatthread(void *ptr); // Real-time EtherCAT thread function

// Thread handles for the EtherCAT threads
OSAL_THREAD_HANDLE thread1; // Handle for the EtherCAT check thread
OSAL_THREAD_HANDLE thread2; // Handle for the real-time EtherCAT thread

// Function to synchronize time with the EtherCAT distributed clock
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime);
// Function to add nanoseconds to a timespec structure
void add_timespec(struct timespec *ts, int64 addtime);

// Define constants for stack size and timing
#define stack64k (64 * 1024) // Stack size for threads
#define NSEC_PER_SEC 1000000000   // Number of nanoseconds in one second
#define EC_TIMEOUTMON 5000        // Timeout for monitoring in microseconds
#define MAX_VELOCITY 30000        // 降低最大速度 (从200000降到30000)
#define MAX_ACCELERATION 50000    // 降低最大加速度 (从500000降到50000)

// Conversion units from the servomotor
float Cnt_to_deg = 0.000686645; // Conversion factor from counts to degrees
int8_t SLAVE_ID; // Slave ID for EtherCAT communication

// Structure for RXPDO (Control data sent to slave)
typedef struct {
    uint16_t controlword;      // 0x6040:0, 16 bits
    int32_t target_position;   // 0x607A:0, 32 bits
    uint8_t mode_of_operation; // 0x6060:0, 8 bits
    uint8_t padding;          // 8 bits padding for alignment
} __attribute__((__packed__)) rxpdo_t;

// Structure for TXPDO (Status data received from slave)
typedef struct {
    uint16_t statusword;      // 0x6041:0, 16 bits
    int32_t actual_position;  // 0x6064:0, 32 bits
    int32_t actual_velocity;  // 0x606C:0, 32 bits
    int16_t actual_torque;    // 0x6077:0, 16 bits
} __attribute__((__packed__)) txpdo_t;

// Add these global variables after the other global declarations
volatile int target_position = 0;
pthread_mutex_t target_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t target_position_cond = PTHREAD_COND_INITIALIZER;
bool target_updated = false;
int32_t received_target = 0;

// 在文件开头的全局变量声明部分添加
rxpdo_t rxpdo;  // 全局变量，用于发送到从站
txpdo_t txpdo;  // 全局变量，用于从从站接收

// 在全局变量区域添加
struct MotorStatus {
    bool is_operational;
    uint16_t status_word;
    int32_t actual_position;
    int32_t actual_velocity;
    int16_t actual_torque;
} motor_status;

// 在文件开头的函数原型声明部分添加（和其他函数原型放在一起）
void update_motor_status(int slave_id);  // 添加函数声明

// 在文件开头，其他宏定义之后添加
#undef MAX_VELOCITY  // 确保没有命名冲突
#undef MAX_ACCELERATION

// 在全局变量声明区域添加
struct MotionPlanner {
    int32_t start_position;    // 起始位置
    int32_t target_position;   // 最终目标位置
    int32_t smooth_target;     // 平滑过渡的当前目标位置
    int32_t current_position;  // 当前规划位置
    double current_velocity;   // 当前速度
    double start_time;         // 开始时间
    double total_time;         // 总时间
    double current_time;       // 当前时间
    bool is_moving;           // 运动状态
    
    // 运动参数
    static constexpr double MAX_VELOCITY = 50000.0;     // 最大速度限制
    static constexpr double CYCLE_TIME = 0.0005;         // 周期时间 1ms
    static constexpr double SMOOTH_FACTOR = 0.002;      // 目标位置平滑因子

    // 五次多项式系数
    double a0, a1, a2, a3, a4, a5;

    MotionPlanner() : start_position(0), target_position(0), smooth_target(0),
                     current_position(0), current_velocity(0.0),
                     start_time(0.0), total_time(0.0), current_time(0.0),
                     is_moving(false) {}
};

// 定义静态成员变量
constexpr double MotionPlanner::MAX_VELOCITY;
constexpr double MotionPlanner::CYCLE_TIME;
constexpr double MotionPlanner::SMOOTH_FACTOR;

// 全局变量
MotionPlanner g_motion_planner;

// 函数声明
int32_t plan_trajectory(MotionPlanner* planner, int32_t actual_position);

//##################################################################################################
// Function: Set the CPU affinity for a thread
void set_thread_affinity(pthread_t thread, int cpu_core) {
    cpu_set_t cpuset; // CPU set to specify which CPUs the thread can run on
    CPU_ZERO(&cpuset); // Clear the CPU set
    CPU_SET(cpu_core, &cpuset); // Add the specified CPU core to the set

    // Set the thread's CPU affinity
    int result = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (result != 0) {
        printf("Unable to set CPU affinity for thread %d\n", cpu_core); // Error message if setting fails
    } else {
        printf("Thread successfully bound to CPU %d\n", cpu_core); // Confirmation message if successful
    }
}

//##################################################################################################
// Function prototype for the EtherCAT test function
int erob_test();

uint16_t data_R;

int erob_test() {
    int rdl; // Variable to hold read data length
    SLAVE_ID = 1; // Set the slave ID to 1
    int i, j, oloop, iloop, chk; // Loop control variables

    // 1. Call ec_config_init() to move from INIT to PRE-OP state.
    printf("__________STEP 1___________________\n");
    // Initialize EtherCAT master on the specified network interface
    if (ec_init("enp6s0") <= 0) {
        printf("Error: Could not initialize EtherCAT master!\n");
        printf("No socket connection on Ethernet port. Execute as root.\n");
        printf("___________________________________________\n");
        return -1; // Return error if initialization fails
    }
    printf("EtherCAT master initialized successfully.\n");
    printf("___________________________________________\n");

    // Search for EtherCAT slaves on the network
    if (ec_config_init(FALSE) <= 0) {
        printf("Error: Cannot find EtherCAT slaves!\n");
        printf("___________________________________________\n");
        ec_close(); // Close the EtherCAT connection
        return -1; // Return error if no slaves are found
    }
    printf("%d slaves found and configured.\n", ec_slavecount); // Print the number of slaves found
    printf("___________________________________________\n");

    // 2. Change to pre-operational state to configure the PDO registers
    printf("__________STEP 2___________________\n");

    // Check if the slave is ready to map
    ec_readstate(); // Read the state of the slaves

    for(int i = 1; i <= ec_slavecount; i++) { // Loop through each slave
        if(ec_slave[i].state != EC_STATE_PRE_OP) { // If the slave is not in PRE-OP state
            // Print the current state and status code of the slave
            printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                   i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            printf("\nRequest init state for slave %d\n", i); // Request to change the state to INIT
            ec_slave[i].state = EC_STATE_INIT; // Set the slave state to INIT
            printf("___________________________________________\n");
        } else { // If the slave is in PRE-OP state
            ec_slave[0].state = EC_STATE_PRE_OP; // Set the first slave to PRE-OP state
            /* Request EC_STATE_PRE_OP state for all slaves */
            ec_writestate(0); // Write the state change to the slave
            /* Wait for all slaves to reach the PRE-OP state */
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

    // Clear RXPDO mapping
    int retval = 0; // Variable to hold the return value of SDO write operations
    uint16 map_1c12; // Variable to hold the mapping for PDO
    uint8 zero_map = 0; // Variable to clear the PDO mapping
    uint32 map_object; // Variable to hold the mapping object
    uint16 clear_val = 0x0000; // Value to clear the mapping

    for(int i = 1; i <= ec_slavecount; i++) { // Loop through each slave
        // 1. First, disable PDO
        retval += ec_SDOwrite(i, 0x1600, 0x00, FALSE, sizeof(zero_map), &zero_map, EC_TIMEOUTSAFE);
        
        // 2. Configure new PDO mapping
        // Control Word
        map_object = 0x60400010;  // 0x6040:0 Control Word, 16 bits
        retval += ec_SDOwrite(i, 0x1600, 0x01, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Target Position
        map_object = 0x607A0020;  // 0x607A:0 Target Position, 32 bits
        retval += ec_SDOwrite(i, 0x1600, 0x02, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Mode of Operation
        map_object = 0x60600008;  // 0x6060:0 Mode of Operation, 8 bits
        retval += ec_SDOwrite(i, 0x1600, 0x03, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Padding (8 bits)
        map_object = 0x00000008;  // 8 bits padding
        retval += ec_SDOwrite(i, 0x1600, 0x04, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        
        // Set number of mapped objects
        uint8 map_count = 4;
        retval += ec_SDOwrite(i, 0x1600, 0x00, FALSE, sizeof(map_count), &map_count, EC_TIMEOUTSAFE);
        
        // 4. Configure RXPDO allocation
        clear_val = 0x0000; // Clear the mapping
        retval += ec_SDOwrite(i, 0x1c12, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);
        map_1c12 = 0x1600; // Set the mapping to the new PDO
        retval += ec_SDOwrite(i, 0x1c12, 0x01, FALSE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
        map_1c12 = 0x0001; // Set the mapping index
        retval += ec_SDOwrite(i, 0x1c12, 0x00, FALSE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
    }

    printf("PDO mapping configuration result: %d\n", retval);
    if (retval < 0) {
        printf("PDO mapping failed\n");
        return -1;
    }

    printf("RXPOD Mapping set correctly.\n");
    printf("___________________________________________\n");

    //........................................................................................
    // Map TXPOD
    retval = 0;
    uint16 map_1c13;
    for(int i = 1; i <= ec_slavecount; i++) {
        // First, clear the TXPDO mapping
        clear_val = 0x0000;
        retval += ec_SDOwrite(i, 0x1A00, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);

        // Configure TXPDO mapping entries
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

        // Set the number of mapped objects (4 objects)
        uint8 map_count = 4;
        retval += ec_SDOwrite(i, 0x1A00, 0x00, FALSE, sizeof(map_count), &map_count, EC_TIMEOUTSAFE);

        // Configure TXPDO assignment
        // First, clear the assignment
        clear_val = 0x0000;
        retval += ec_SDOwrite(i, 0x1C13, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);

        // Assign TXPDO to 0x1A00
        map_1c13 = 0x1A00;
        retval += ec_SDOwrite(i, 0x1C13, 0x01, FALSE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);

        // Set the number of assigned PDOs (1 PDO)
        map_1c13 = 0x0001;
        retval += ec_SDOwrite(i, 0x1C13, 0x00, FALSE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);
    }

    printf("Slave %d TXPDO mapping configuration result: %d\n", SLAVE_ID, retval);

    if (retval < 0) {
        printf("TXPDO Mapping failed\n");
        printf("___________________________________________\n");
        return -1;
    }

    printf("TXPDO Mapping set successfully\n");
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
        ecx_dcsync0(&ecx_context, i, TRUE, 500000, 0);  //Synchronize the distributed clock for the slave
    }

    // Map the configured PDOs to the IOmap
    ec_config_map(&IOmap);

    printf("__________STEP 5___________________\n");

    // Ensure all slaves are in PRE-OP state
    for(int i = 1; i <= ec_slavecount; i++) {
        if(ec_slave[i].state != EC_STATE_PRE_OP) { // Check if the slave is not in PRE-OP state
            printf("Slave %d not in PRE-OP state. Current state: %d\n", i, ec_slave[i].state);
            return -1; // Return error if any slave is not in PRE-OP state
        }
    }

    // Configure Distributed Clock (DC)
    ec_configdc(); // Set up the distributed clock for synchronization

    // Request to switch to SAFE-OP state
    ec_slave[0].state = EC_STATE_SAFE_OP; // Set the first slave to SAFE-OP state
    ec_writestate(0); // Write the state change to the slave

    // Wait for the state transition
    if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) == EC_STATE_SAFE_OP) {
        printf("Successfully changed to SAFE_OP state\n"); // Confirm successful state change
    } else {
        printf("Failed to change to SAFE_OP state\n");
        return -1; // Return error if state change fails
    }

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

    // Send process data to the slaves
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET); // Receive process data and store the Work Counter

    // Set the first slave to operational state
    ec_slave[0].state = EC_STATE_OPERATIONAL; // Change the state of the first slave to OP
    ec_writestate(0); // Write the state change to the slave

    // Wait for the state transition to complete
    if ((ec_statecheck(0, EC_STATE_OPERATIONAL, 5 * EC_TIMEOUTSTATE)) == EC_STATE_OPERATIONAL) {
        printf("State changed to EC_STATE_OPERATIONAL: %d\n", EC_STATE_OPERATIONAL); // Confirm successful state change
        printf("___________________________________________\n");
    } else {
        printf("State could not be changed to EC_STATE_OPERATIONAL\n"); // Error message if state change fails
        for (int cnt = 1; cnt <= ec_slavecount; cnt++) {
            printf("ALstatuscode: %d\n", ecx_context.slavelist[cnt].ALstatuscode); // Print AL status codes for each slave
        }
    }

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
        printf("Operational state reached for all slaves.\n");
        
        uint8 operation_mode = 8;  // PP Mode = 1
        
        // 初始化PDO数据
        rxpdo_t rxpdo;
        txpdo_t txpdo;
        
        uint16_t  Control_Word;
        Control_Word = 128;
        

        uint32_t  Profile_velocity;
        Profile_velocity = 50000;

        uint32_t  Profile_acceleration;
        Profile_acceleration = 150000;

        uint32_t  Profile_deceleration;
        Profile_deceleration = 150000;
        


        for (int i = 1; i <= ec_slavecount; i++) {
            ec_SDOwrite(i, 0x6040, 0x00, FALSE, sizeof(Control_Word), &Control_Word, EC_TIMEOUTSAFE);
            ec_SDOwrite(i, 0x6060, 0x00, FALSE, sizeof(operation_mode), &operation_mode, EC_TIMEOUTSAFE);
            ec_SDOwrite(i, 0x6081, 0x00, FALSE, sizeof(Profile_velocity), &Profile_velocity, EC_TIMEOUTSAFE);
            ec_SDOwrite(i, 0x6083, 0x00, FALSE, sizeof(Profile_acceleration), &Profile_acceleration, EC_TIMEOUTSAFE);
            ec_SDOwrite(i, 0x6084, 0x00, FALSE, sizeof(Profile_deceleration), &Profile_deceleration, EC_TIMEOUTSAFE);

        }
        // 记录开始时间
        auto start = std::chrono::high_resolution_clock::now();
        int step = 0;

        // 主循环
        for(i = 1; i <= 3 * 60 * 60 * 1000; i++) {
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);

            

            // Send output data to each slave
            for (int slave = 1; slave <= ec_slavecount; slave++) {
                memcpy(ec_slave[slave].outputs, &rxpdo, sizeof(rxpdo_t));
            }

            if(wkc >= expectedWKC) {
                if (i % 100 == 0) {  // Print every 100 cycles
                    for(int slave = 1; slave <= ec_slavecount; slave++) {
                        // Get input data using the structure
                        memcpy(&txpdo, ec_slave[slave].inputs, sizeof(txpdo_t));
                        
                        // Print received data
                        printf("Slave %d:\n", slave);
                        printf("  Status Word: 0x%04x\n", txpdo.statusword);
                        printf("  Position: %d\n", txpdo.actual_position);
                        printf("  Velocity: %d\n", txpdo.actual_velocity);
                        printf("  Torque: %d\n", txpdo.actual_torque);
                        
                        // Parse status word
                        printf("  State: ");
                        if (txpdo.statusword & 0x0001) printf("Ready to switch on ");
                        if (txpdo.statusword & 0x0002) printf("Switched on ");
                        if (txpdo.statusword & 0x0004) printf("Operation enabled ");
                        if (txpdo.statusword & 0x0008) printf("Fault ");
                        if (txpdo.statusword & 0x0010) printf("Voltage enabled ");
                        if (txpdo.statusword & 0x0020) printf("Quick stop ");
                        if (txpdo.statusword & 0x0040) printf("Switch on disabled ");
                        if (txpdo.statusword & 0x0080) printf("Warning ");
                        printf("\n");
                    }
                    printf("----------------------------------------\n");
                    
                }
                needlf = TRUE;

           if (step <= 200) {
                // Initial state
                rxpdo.controlword = 0x0080;
                rxpdo.target_position = 0;
                rxpdo.mode_of_operation = 8;
                rxpdo.padding = 0;
            }
            else if (step <= 300) {
                // Shutdown command
                rxpdo.controlword = 0x0006;
                rxpdo.target_position = txpdo.actual_position;
                rxpdo.mode_of_operation = 8;
                rxpdo.padding = 0;
            }
            else if (step <= 400) {
                // Switch On command
                rxpdo.controlword = 0x0007;
                rxpdo.target_position = txpdo.actual_position;
                rxpdo.mode_of_operation = 8;
                rxpdo.padding = 0;
            }
            else if (step <= 500) {
                // Enable Operation command
                rxpdo.controlword = 0x00F;
                rxpdo.target_position = txpdo.actual_position;
                rxpdo.mode_of_operation = 8;
                rxpdo.padding = 0;
            }
            else {
                // Normal operation with position control
                update_motor_status(SLAVE_ID);  // Update motor status
                
                // 只在更新目标位置时使用互斥锁
                pthread_mutex_lock(&target_mutex);
                bool need_update = target_updated;
                int32_t new_target = received_target;
                if (need_update) {
                    target_updated = false;
                }
                pthread_mutex_unlock(&target_mutex);

                printf("\n当前状态:\n");
                printf("当前位置: %d\n", txpdo.actual_position);
                printf("目标位置: %d\n", new_target);
                printf("更新标志: %d\n", need_update);
                
                if (need_update) {
                    g_motion_planner.target_position = new_target;
                    printf(">>> 运动规划器更新:\n");
                    printf("  新目标位置: %d\n", new_target);
                    printf("  规划器目标: %d\n", g_motion_planner.target_position);
                    printf("  当前位置: %d\n", txpdo.actual_position);
                    g_motion_planner.is_moving = true;  // 确保开始运动
                }

                // 进行运动规划
                int32_t planned_pos = plan_trajectory(&g_motion_planner, txpdo.actual_position);
                printf("规划结果: current=%d, planned=%d, target=%d, moving=%d\n",
                       txpdo.actual_position, planned_pos, g_motion_planner.target_position,
                       g_motion_planner.is_moving);
                
                rxpdo.target_position = planned_pos;

                // 每次循环都打印状态
                printf("运动状态:\n");
                printf("  控制字: 0x%04x\n", rxpdo.controlword);
                printf("  状态字: 0x%04x\n", txpdo.statusword);
                printf("  运行模式: %d\n", rxpdo.mode_of_operation);
                printf("  位置误差: %d\n", g_motion_planner.target_position - txpdo.actual_position);
                printf("--------------------\n");
            }
            }

            if(step < 900) {
                step += 1;
            }

            osal_usleep(500);
        }
    }

    osal_usleep(1e6);

    ec_close();

     printf("\nRequest init state for all slaves\n");
     ec_slave[0].state = EC_STATE_INIT;
     /* request INIT state for all slaves */
     ec_writestate(0);

    printf("EtherCAT master closed.\n");

    return 0;
}

/* 
 * PI calculation to synchronize Linux time with the Distributed Clock (DC) time.
 * This function calculates the offset time needed to align the Linux time with the DC time.
 */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime) {
    static int64 integral = 0; // Integral term for PI controller
    int64 delta; // Variable to hold the difference between reference time and cycle time
    delta = (reftime) % cycletime; // Calculate the delta time
    if (delta > (cycletime / 2)) {
        delta = delta - cycletime; // Adjust delta if it's greater than half the cycle time
    }
    if (delta > 0) {
        integral++; // Increment integral if delta is positive
    }
    if (delta < 0) {
        integral--; // Decrement integral if delta is negative
    }
    *offsettime = -(delta / 100) - (integral / 20); // Calculate the offset time
    gl_delta = delta; // Update global delta variable
}

/* 
 * Add nanoseconds to a timespec structure.
 * This function updates the timespec structure by adding a specified amount of time.
 */
void add_timespec(struct timespec *ts, int64 addtime) {
    int64 sec, nsec; // Variables to hold seconds and nanoseconds

    nsec = addtime % NSEC_PER_SEC; // Calculate nanoseconds to add
    sec = (addtime - nsec) / NSEC_PER_SEC; // Calculate seconds to add
    ts->tv_sec += sec; // Update seconds in timespec
    ts->tv_nsec += nsec; // Update nanoseconds in timespec
    if (ts->tv_nsec >= NSEC_PER_SEC) { // If nanoseconds exceed 1 second
        nsec = ts->tv_nsec % NSEC_PER_SEC; // Adjust nanoseconds
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC; // Increment seconds
        ts->tv_nsec = nsec; // Set adjusted nanoseconds
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

    while (1) { // Infinite loop for monitoring
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)) {
            if (needlf) {
                needlf = FALSE; // Reset line feed flag
                printf("\n"); // Print a new line
            }
            ec_group[currentgroup].docheckstate = FALSE; // Reset check state
            ec_readstate(); // Read the state of all slaves
            for (slave = 1; slave <= ec_slavecount; slave++) { // Loop through each slave
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
                    ec_group[currentgroup].docheckstate = TRUE; // Set check state if slave is not operational
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                        printf("ERROR: Slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK); // Acknowledge error state
                        ec_writestate(slave); // Write the state change to the slave
                    } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
                        printf("WARNING: Slave %d is in SAFE_OP, changing to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL; // Change state to operational
                        ec_writestate(slave); // Write the state change to the slave
                    } else if (ec_slave[slave].state > EC_STATE_NONE) {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) { // Reconfigure the slave if needed
                            ec_slave[slave].islost = FALSE; // Mark slave as found
                            printf("MESSAGE: Slave %d reconfigured\n", slave);
                        }
                    } else if (!ec_slave[slave].islost) {
                        ec_statecheck(slave, EC_STATE_OPERATIONAL,  EC_TIMEOUTRET); // Check the state of the slave
                        if (ec_slave[slave].state == EC_STATE_NONE) {
                            ec_slave[slave].islost = TRUE; // Mark slave as lost
                            printf("ERROR: Slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost) { // If the slave is marked as lost
                    if (ec_slave[slave].state == EC_STATE_NONE) {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON)) { // Attempt to recover the lost slave
                            ec_slave[slave].islost = FALSE; // Mark slave as found
                            printf("MESSAGE: Slave %d recovered\n", slave);
                        }
                    } else {
                        ec_slave[slave].islost = FALSE; // Mark slave as found
                        printf("MESSAGE: Slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate) {
                printf("OK: All slaves resumed OPERATIONAL.\n"); // Confirm all slaves are operational
            }
        }
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
    struct timespec ts, tleft; // Variables for time management
    int ht; // Variable for high-resolution time
    int64 cycletime; // Variable to hold the cycle time
    struct timeval tp; // Variable for time value

    // Get the current time in monotonic clock
    clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1; /* Round to nearest ms */
    ts.tv_nsec = ht * 1000000; // Set nanoseconds to the rounded value
    if (ts.tv_nsec >= NSEC_PER_SEC) { // If nanoseconds exceed 1 second
        ts.tv_sec++; // Increment seconds
        ts.tv_nsec -= NSEC_PER_SEC; // Adjust nanoseconds
    }
    cycletime = *(int *)ptr * 1000; /* Convert cycle time from ms to ns */

    toff = 0; // Initialize time offset
    dorun = 0; // Initialize run flag
    ec_send_processdata(); // Send initial process data

    while (1) { // Infinite loop for real-time processing
        dorun++; // Increment run counter
        /* Calculate next cycle start */
        add_timespec(&ts, cycletime + toff); // Add cycle time to the current time
        /* Wait for the cycle start */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft); // Sleep until the next cycle

        if (start_ecatthread_thread) { // Check if the EtherCAT thread should run
            wkc = ec_receive_processdata(EC_TIMEOUTRET); // Receive process data and store the Work Counter

            if (ec_slave[0].hasdc) { // If the first slave supports Distributed Clock
                /* Calculate toff to synchronize Linux time with DC time */
                ec_sync(ec_DCtime, cycletime, &toff); // Synchronize time
            }

            ec_send_processdata(); // Send process data to the slaves
        }
    }
}

int correct_count = 0;
int incorrect_count = 0;
int test_count_sum = 100;
int test_count = 0;
float correct_rate = 0;

// 在 MotionPlanner 结构体定义之前添加辅助函数
template<typename T>
T clamp(T value, T min_val, T max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// 平滑更新目标位置
int32_t update_smooth_target(MotionPlanner* planner) {
    double position_diff = planner->target_position - planner->smooth_target;
    
    // 根据距离动态调整平滑因子
    double adaptive_factor = planner->SMOOTH_FACTOR;

    
    // 计算本周期的位置增量
    double position_increment = position_diff * adaptive_factor;
    
    // 限制每个周期的位置变化量
    double max_increment = planner->MAX_VELOCITY * planner->CYCLE_TIME;
    if (fabs(position_increment) > max_increment) {
        position_increment = copysign(max_increment, position_increment);
    }
    
    // 更新平滑目标位置
    planner->smooth_target += position_increment;
    
    return planner->smooth_target;
}

// 五次多项式轨迹规划
int32_t plan_trajectory(MotionPlanner* planner, int32_t actual_position) {
    // 首次启动时初始化
    if (!planner->is_moving) {
        planner->start_position = actual_position;
        planner->current_position = actual_position;
        planner->smooth_target = actual_position;
        planner->current_velocity = 0.0;
        planner->current_time = 0.0;
        planner->is_moving = true;
    }
    
    // 平滑更新目标位置
    int32_t current_target = update_smooth_target(planner);
    
    // 计算到最终目标的距离
    double final_distance = planner->target_position - planner->current_position;
    
    // 如果非常接近最终目标位置，进入精确定位模式
    if (fabs(final_distance) < 100) {
        // 使用比例控制进行精确定位
        double position_error = planner->target_position - planner->current_position;
        double precise_velocity = position_error * 0.5;  // 比例因子0.5
        
        // 限制精确定位时的速度
        precise_velocity = clamp(precise_velocity, -100.0, 100.0);
        
        // 如果误差很小，直接到达目标位置
        if (fabs(position_error) < fabs(precise_velocity * planner->CYCLE_TIME)) {
            planner->current_position = planner->target_position;
            planner->current_velocity = 0.0;
            planner->is_moving = false;
            return planner->target_position;
        }
        
        // 更新位置
        planner->current_position += precise_velocity * planner->CYCLE_TIME;
        planner->current_velocity = precise_velocity;
        return static_cast<int32_t>(planner->current_position);
    }
    
    // 正常的轨迹规划
    if (current_target != planner->current_position) {
        planner->start_position = planner->current_position;
        double distance = fabs(current_target - planner->start_position);
        
        // 根据距离计算合适的运动时间
        double min_time = 2.0 * distance / planner->MAX_VELOCITY;
        planner->total_time = min_time * 1.5;
        planner->current_time = 0.0;
        
        // 重新计算多项式系数
        double T = planner->total_time;
        double x0 = planner->start_position;
        double xf = current_target;
        double v0 = planner->current_velocity;
        double vf = 0;
        double a0 = 0;
        double af = 0;

        planner->a0 = x0;
        planner->a1 = v0;
        planner->a2 = a0/2.0;
        planner->a3 = (20*xf - 20*x0 - (8*vf + 12*v0)*T - (3*a0 - af)*T*T)/(2*T*T*T);
        planner->a4 = (30*x0 - 30*xf + (14*vf + 16*v0)*T + (3*a0 - 2*af)*T*T)/(2*T*T*T*T);
        planner->a5 = (12*xf - 12*x0 - (6*vf + 6*v0)*T - (a0 - af)*T*T)/(2*T*T*T*T*T);
    }

    // 更新时间
    planner->current_time += planner->CYCLE_TIME;
    double t = planner->current_time;

    if (t >= planner->total_time) {
        // 继续运动而不是直接设置位置
        planner->current_time = planner->total_time;
        t = planner->total_time;
    }

    // 计算当前位置和速度
    planner->current_position = planner->a0 + 
                               planner->a1 * t + 
                               planner->a2 * t * t + 
                               planner->a3 * t * t * t + 
                               planner->a4 * t * t * t * t + 
                               planner->a5 * t * t * t * t * t;

    planner->current_velocity = planner->a1 + 
                               2 * planner->a2 * t + 
                               3 * planner->a3 * t * t + 
                               4 * planner->a4 * t * t * t + 
                               5 * planner->a5 * t * t * t * t;

    // 确保速度不超过限制
    planner->current_velocity = clamp(planner->current_velocity, 
                                    -planner->MAX_VELOCITY, 
                                     planner->MAX_VELOCITY);

    // 调试信息
    if (rand() % 1000 == 0) {
        printf("\nTrajectory Debug:\n");
        printf("Target: %d, Current: %.2f\n", planner->target_position, planner->current_position);
        printf("Final Distance: %.2f\n", final_distance);
        printf("Velocity: %.2f\n", planner->current_velocity);
    }

    return static_cast<int32_t>(planner->current_position);
}

// Add the server function before erob_test()
void* start_server(void* arg) {
    int port = *(int*)arg;
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};

    // Create socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        std::cerr << "Socket creation failed" << std::endl;
        return nullptr;
    }

    // Set socket options
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
        std::cerr << "Failed to set socket options" << std::endl;
        return nullptr;
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    // Bind socket
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        std::cerr << "Binding failed" << std::endl;
        return nullptr;
    }

    // Listen for connections
    if (listen(server_fd, 3) < 0) {
        std::cerr << "Listening failed" << std::endl;
        return nullptr;
    }

    std::cout << "Server started, waiting for connection on port " << port << "..." << std::endl;
    
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
        std::cerr << "Failed to accept connection" << std::endl;
        return nullptr;
    }
    std::cout << "Client connected!" << std::endl;

    // Receive data
    while (true) {
        int valread = read(new_socket, buffer, 1024);
        if (valread > 0) {
            try {
                std::cout << "\n收到原始数据: " << buffer << std::endl;
                int32_t new_target = std::stoi(buffer);
                
                pthread_mutex_lock(&target_mutex);
                received_target = new_target;
                target_updated = true;
                std::cout << "设置新目标位置: " << new_target << std::endl;
                std::cout << "更新标志设置为: " << target_updated << std::endl;
                pthread_mutex_unlock(&target_mutex);
            } catch (const std::exception& e) {
                std::cerr << "数据转换错误: " << e.what() << std::endl;
            }
            memset(buffer, 0, sizeof(buffer));
        } else if (valread == 0) {
            std::cout << "Client disconnected" << std::endl;
            break;
        } else {
            std::cerr << "Read error: " << strerror(errno) << std::endl;
        }
    }

    close(new_socket);
    close(server_fd);
    std::cout << "Server closed" << std::endl;
    return nullptr;
}

// Function to update motor status information
void update_motor_status(int slave_id) {
    // Update status information from TXPDO
    motor_status.status_word = txpdo.statusword;
    motor_status.actual_position = txpdo.actual_position;
    motor_status.actual_velocity = txpdo.actual_velocity;
    motor_status.actual_torque = txpdo.actual_torque;
    
    // Check status word to determine if motor is operational
    // Bits 0-3 should be 0111 for enabled and ready state
    motor_status.is_operational = (txpdo.statusword & 0x0F) == 0x07;
}

// Modify the main function to start the server thread
int main(int argc, char **argv) {
    needlf = FALSE;
    inOP = FALSE;
    start_ecatthread_thread = FALSE;
    dorun = 0;
    ctime_thread = 1000; // 1ms cycle time

    // Set CPU affinity
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);

    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) {
        perror("sched_setaffinity");
        return EXIT_FAILURE;
    }

    // Start the server thread
    pthread_t server_thread;
    int port = 8080;  // You can change this port number
    if (pthread_create(&server_thread, NULL, start_server, &port) != 0) {
        std::cerr << "Failed to create server thread" << std::endl;
        return EXIT_FAILURE;
    }

    printf("Running on CPU core 3\n");
    erob_test();
    printf("End program\n");

    return EXIT_SUCCESS;
}

