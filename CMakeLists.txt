cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(test_project C CXX ASM)

set(CMAKE_C_STANDARD 11)

set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(picow_as3935 picow_as3935.cpp as3935.cpp)

pico_enable_stdio_uart(picow_as3935 1)

target_link_libraries(picow_as3935
        pico_stdlib              # for core functionality
        pico_cyw43_arch_none     # we need Wifi to access the GPIO, but we don't need anything else
        hardware_spi
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(picow_as3935)

# add url via pico_set_program_url

