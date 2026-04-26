cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Gimbal_um)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        User/Bsp/Src/bsp_usb.c
        User/Bsp/Src/bsp_imu.c
        User/Bsp/Src/bsp_can.c
        User/Library/MahonyAHRS.c
        User/Device/device_gm6020.cpp
        User/Bsp/Inc/bsp_can.h
        User/Module/MotorManage.cpp
        User/Module/PID.cpp
        User/Module/PID.h
        User/Task/ctrl_task.cpp
        User/Task/ctrl_msg_queue.c
        User/Task/communicate_task.cpp
        User/Task/debug_tune.cpp
        User/Task/debug_tune.h
        User/Task/rtt_task.cpp
        User/Module/MotorManage.h
        User/Module/imu_fusion.cpp
        User/Task/debug_task.cpp
        User/Library/SEGGER_RTT.c
        User/Library/SEGGER_RTT_printf.c)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    User/Bsp/Inc
    User/Device
    User/Library
    User/Module
    User/Task

)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

set(OPENOCD_TOOL_ROOT "${CMAKE_SOURCE_DIR}/tools/openocd")
set(OPENOCD_INTERFACE_CFG "interface/daplink.cfg" CACHE STRING "OpenOCD interface configuration file")
set(OPENOCD_TARGET_CFG "target/stm32f4x.cfg" CACHE STRING "OpenOCD target configuration file")
set(OPENOCD_ADAPTER_SPEED "4000" CACHE STRING "OpenOCD adapter speed in kHz")
set(OPENOCD_FLASH_FILE "$<TARGET_FILE:${CMAKE_PROJECT_NAME}>" CACHE STRING "Firmware image flashed by the flash target")

if(CMAKE_HOST_WIN32)
    set(OPENOCD_EXECUTABLE "${OPENOCD_TOOL_ROOT}/windows/bin/openocd.exe")
    set(OPENOCD_SCRIPTS_DIR "${OPENOCD_TOOL_ROOT}/windows/openocd/scripts")
elseif(CMAKE_HOST_UNIX AND NOT CMAKE_HOST_APPLE)
    set(OPENOCD_EXECUTABLE "${OPENOCD_TOOL_ROOT}/linux/bin/openocd")
    set(OPENOCD_SCRIPTS_DIR "${OPENOCD_TOOL_ROOT}/linux/openocd/scripts")
    set(OPENOCD_LIBRARY_DIR "${OPENOCD_TOOL_ROOT}/linux/libexec")
else()
    message(FATAL_ERROR "Bundled OpenOCD only supports Windows and Linux hosts.")
endif()

if(NOT EXISTS "${OPENOCD_EXECUTABLE}")
    message(FATAL_ERROR
        "Bundled OpenOCD executable not found for this host: ${OPENOCD_EXECUTABLE}\n"
        "Please make sure the repository includes tools/openocd for your platform."
    )
endif()

if(NOT EXISTS "${OPENOCD_SCRIPTS_DIR}")
    message(FATAL_ERROR
        "Bundled OpenOCD scripts directory not found for this host: ${OPENOCD_SCRIPTS_DIR}\n"
        "Please make sure the repository includes the matching OpenOCD scripts."
    )
endif()

set(OPENOCD_FLASH_COMMAND
    "${OPENOCD_EXECUTABLE}"
    -s "${OPENOCD_SCRIPTS_DIR}"
    -f "${OPENOCD_INTERFACE_CFG}"
    -f "${OPENOCD_TARGET_CFG}"
    -c "adapter speed ${OPENOCD_ADAPTER_SPEED}"
    -c "program ${OPENOCD_FLASH_FILE} verify reset exit"
)

if(CMAKE_HOST_UNIX AND NOT CMAKE_HOST_APPLE)
    if(NOT EXISTS "${OPENOCD_LIBRARY_DIR}")
        message(FATAL_ERROR
            "Bundled OpenOCD runtime directory not found for Linux host: ${OPENOCD_LIBRARY_DIR}\n"
            "Please make sure the repository includes the matching OpenOCD runtime libraries."
        )
    endif()

    add_custom_target(flash
        COMMAND "${CMAKE_COMMAND}" -E env
            "LD_LIBRARY_PATH=${OPENOCD_LIBRARY_DIR}:$ENV{LD_LIBRARY_PATH}"
            ${OPENOCD_FLASH_COMMAND}
        DEPENDS ${CMAKE_PROJECT_NAME}
        USES_TERMINAL
        COMMENT "Flashing ${CMAKE_PROJECT_NAME} with bundled OpenOCD"
    )
else()
    add_custom_target(flash
        COMMAND ${OPENOCD_FLASH_COMMAND}
        DEPENDS ${CMAKE_PROJECT_NAME}
        USES_TERMINAL
        COMMENT "Flashing ${CMAKE_PROJECT_NAME} with bundled OpenOCD"
    )
endif()
