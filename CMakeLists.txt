cmake_minimum_required(VERSION 3.9)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules")
project(SOEM
    DESCRIPTION "Simple Open EtherCAT Master"
    VERSION 1.4.0
    LANGUAGES C CXX
)

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)   # Default to installing in SOEM source directory
    set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_LIST_DIR}/install)
endif()

set(SOEM_INCLUDE_INSTALL_DIR include/soem)
set(SOEM_LIB_INSTALL_DIR lib)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(BUILD_TESTS TRUE)
else()
    message(STATUS "SOEM: not building tests when built as dependency")
    set(BUILD_TESTS FALSE)
endif()

if(WIN32)
    set(OS "win32")
    include_directories(oshw/win32/wpcap/Include)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        link_directories(${CMAKE_CURRENT_LIST_DIR}/oshw/win32/wpcap/Lib/x64)
    elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
        link_directories(${CMAKE_CURRENT_LIST_DIR}/oshw/win32/wpcap/Lib)
    endif()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /D _CRT_SECURE_NO_WARNINGS")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  /WX")
    set(OS_LIBS wpcap.lib Packet.lib Ws2_32.lib Winmm.lib)
elseif(UNIX AND NOT APPLE)
    set(OS "linux")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
    set(OS_LIBS pthread rt)
elseif(APPLE) 
    # This must come *before* linux or MacOSX will identify as Unix.
    set(OS "macosx")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
    set(OS_LIBS pthread pcap)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rt-kernel")
    set(OS "rtk")
    message(STATUS "ARCH is ${ARCH}")
    message(STATUS "BSP is ${BSP}")
    include_directories(oshw/${OS}/${ARCH})
    file(GLOB OSHW_EXTRA_SOURCES oshw/${OS}/${ARCH}/*.c)
    set(OSHW_SOURCES "${OS_HW_SOURCES} ${OSHW_ARCHSOURCES}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-but-set-variable")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-function")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-format")
    set(OS_LIBS "-Wl,--start-group -l${BSP} -l${ARCH} -lkern -ldev -lsio -lblock -lfs -lusb -llwip -leth -li2c -lrtc -lcan -lnand -lspi -lnor -lpwm -ladc -ltrace -lc -lm -Wl,--end-group")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rtems")
    message(STATUS "Building for RTEMS")
    set(OS "rtems")
    set(SOEM_LIB_INSTALL_DIR ${LIB_DIR})
    set(BUILD_TESTS FALSE)
endif()

message(STATUS "OS is ${OS}")

file(GLOB SOEM_SOURCES soem/*.c)
file(GLOB OSAL_SOURCES osal/${OS}/*.c)
file(GLOB OSHW_SOURCES oshw/${OS}/*.c)

file(GLOB SOEM_HEADERS soem/*.h)
file(GLOB OSAL_HEADERS osal/osal.h osal/${OS}/*.h)
file(GLOB OSHW_HEADERS oshw/${OS}/*.h)

# Change to SHARED here
add_library(soem SHARED
    ${SOEM_SOURCES}
    ${OSAL_SOURCES}
    ${OSHW_SOURCES}
    ${OSHW_EXTRA_SOURCES}
)

# Link necessary system libraries
target_link_libraries(soem ${OS_LIBS})

# Specify include directories
target_include_directories(soem PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/soem>
    $<INSTALL_INTERFACE:include/soem>
)
target_include_directories(soem PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/osal>
    $<INSTALL_INTERFACE:include/soem>
)
target_include_directories(soem PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/osal/${OS}>
    $<INSTALL_INTERFACE:include/soem>
)
target_include_directories(soem
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/oshw/${OS}>
    $<INSTALL_INTERFACE:include/soem>
)

message(STATUS "LIB_DIR: ${SOEM_LIB_INSTALL_DIR}")

# Install targets
install(TARGETS soem EXPORT soemConfig DESTINATION ${SOEM_LIB_INSTALL_DIR})
install(EXPORT soemConfig DESTINATION share/soem/cmake)

# Install header files
install(FILES
    ${SOEM_HEADERS}
    ${OSAL_HEADERS}
    ${OSHW_HEADERS}
    DESTINATION ${SOEM_INCLUDE_INSTALL_DIR}
)

if(BUILD_TESTS)
    find_package(Threads REQUIRED)
    
    add_subdirectory(test/simple_ng)
    add_subdirectory(test/linux/slaveinfo)
    add_subdirectory(test/linux/eepromtool)
    add_subdirectory(test/linux/simple_test)
    add_subdirectory(demo)
    
    # Add ERob Motor Controller library
    include_directories(include)
    
    # Set C++ standard
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    
    # Source files for the motor controller library
    set(EROB_CONTROLLER_SOURCES
        src/ERobMotorController.cpp
        src/ERobMotorController_Private.cpp
    )
    
    # Create the motor controller library
    add_library(ERobMotorController SHARED ${EROB_CONTROLLER_SOURCES})
    target_link_libraries(ERobMotorController soem pthread)
    target_compile_options(ERobMotorController PRIVATE -Wall -Wextra)
    
    # Install library and headers
    install(TARGETS ERobMotorController 
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
    )
    install(FILES include/ERobMotorController.hpp DESTINATION include)
    
    # Add example executables
    add_executable(three_axis_example examples/ERobThreeAxisExample.cpp)
    target_link_libraries(three_axis_example ERobMotorController soem pthread)
    install(TARGETS three_axis_example DESTINATION bin)
    
    add_executable(controller_test examples/controller_test.cpp)
    target_link_libraries(controller_test ERobMotorController soem pthread)
    install(TARGETS controller_test DESTINATION bin)
endif()
