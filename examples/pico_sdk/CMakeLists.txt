cmake_minimum_required(VERSION 3.13)

if (NOT DEFINED ENV{PICO_SDK_PATH})
    message(FATAL_ERROR "Set PICO_SDK_PATH to the Pico SDK directory before configuring.")
endif()

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(icm45686_pico_spi C CXX ASM)
pico_sdk_init()

add_executable(icm45686_pico_spi
    main.c
)

target_link_libraries(icm45686_pico_spi
    pico_stdlib
    hardware_spi
)

pico_enable_stdio_usb(icm45686_pico_spi 1)
pico_enable_stdio_uart(icm45686_pico_spi 0)
pico_add_extra_outputs(icm45686_pico_spi)
