cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

set(target_proj MS70CTRL)
project(${target_proj} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

include(${PICO_SDK_PATH}/lib/tinyusb/hw/bsp/family_support.cmake)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
add_subdirectory(./libs/RP2040-Button button)
add_subdirectory(./libs/usb_midi_host usb_midi_host) 

# Tell CMake where to find the executable source file
add_executable(${target_proj} 
    main.c
)

target_include_directories(${target_proj} PRIVATE
 ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(${target_proj} tinyusb_host tinyusb_board usb_midi_host_app_driver pico_stdlib button)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${target_proj})