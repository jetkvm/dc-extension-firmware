# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(jetkvm-dc C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name.

add_executable(jetkvm-dc jetkvm-dc.c driver_ina219.c flash_store.c)

pico_set_program_name(jetkvm-dc "jetkvm-dc")
pico_set_program_version(jetkvm-dc "0.2.0")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(jetkvm-dc 0)
pico_enable_stdio_usb(jetkvm-dc 1)

# Add the standard library to the build
target_link_libraries(jetkvm-dc
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(jetkvm-dc PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(jetkvm-dc 
        hardware_watchdog
        hardware_i2c
        )

pico_add_extra_outputs(jetkvm-dc)

#The default XOSC startup time is 1ms, which is too short for our XOSC
add_compile_definitions(
    PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64
)