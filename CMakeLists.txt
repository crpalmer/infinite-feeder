cmake_minimum_required(VERSION 3.13)

include(lib/platform.cmake)

# Stop Pico SDK from trying to fetch/build picotool on this machine
set(PICO_SDK_FETCH_PICOTOOL OFF)

project(infinite-feeder C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(lib)

add_executable(firmware firmware.cpp
    uart-channel.cpp
)

target_link_libraries(firmware 
    lib-pi
    lib-pi-threads
)

pico_enable_stdio_usb(firmware 1)
pico_enable_stdio_uart(firmware 0)
pico_add_extra_outputs(firmware)

add_executable(pico pico.cpp
    uart-channel.cpp
)

target_link_libraries(pico
    lib-pi
    lib-pi-threads
)

pico_enable_stdio_usb(pico 1)
pico_enable_stdio_uart(pico 0)
pico_add_extra_outputs(pico)

add_executable(test test.cpp
)

target_link_libraries(test 
    lib-pi
    lib-pi-threads
)

pico_enable_stdio_usb(test 1)
pico_enable_stdio_uart(test 0)
pico_add_extra_outputs(test)
