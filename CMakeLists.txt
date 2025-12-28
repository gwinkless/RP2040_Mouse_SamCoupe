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
set(CMAKE_SYSTEM_NAME Generic)

cmake_minimum_required(VERSION 3.13)
set(PICO_SDK_FETCH_FROM_GIT on)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(rp2040_mouse)
enable_language( C CXX ASM )
pico_sdk_init()

set(target_name rp2040_mouse)
add_executable(${target_name})

pico_generate_pio_header(${target_name} ${CMAKE_CURRENT_LIST_DIR}/sam_mouse.pio)
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

target_link_libraries(${target_name} PRIVATE pico_stdlib pico_multicore hardware_pio tinyusb_host tinyusb_board)
pico_add_extra_outputs(${target_name})