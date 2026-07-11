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
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")
set(PICO_TINYUSB_PATH "C:/Users/def_g/GITROOT/tinyusb")
# Tell TinyUSB not to use its own RP2040 BSP
add_compile_definitions(CFG_TUH_HID_SET_PROTOCOL_ON_ENUM=1)
include_directories(${PICO_TINYUSB_PATH}/src ${PICO_TINYUSB_PATH})
include(pico_sdk_import.cmake)
project(rp2040_mouse C CXX ASM )

pico_sdk_init()

set(target_name rp2040_mouse)
add_executable(${target_name})

target_sources(${target_name} PRIVATE
 main.c
 )

 pico_enable_stdio_uart(${target_name} 1)
 pico_enable_stdio_usb(${target_name} 0)

# set info for picotool

pico_set_program_name(rp2040_mouse "RP2040 Mouse")
pico_set_program_description(rp2040_mouse "RP2040 Based USB to Sam Coupe mouse adaptor")
pico_set_program_version(rp2040_mouse "1.0")
pico_set_program_url(rp2040_mouse "https://github.com/gwinkless/RP2040_Mouse_SamCoupe")
 
# print memory usage, enable all warnings
target_link_options(${target_name} PRIVATE -Xlinker --print-memory-usage)
target_compile_options(${target_name} PRIVATE -Wall -Wextra)
# needed so tinyusb can find tusb_config.h
target_include_directories(${target_name} PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${target_name} PRIVATE pico_stdlib pico_multicore tinyusb_host tinyusb_board)
pico_add_extra_outputs(${target_name})