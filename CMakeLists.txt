cmake_minimum_required(VERSION 3.13)

include(lib/platform.cmake)

function(pico_specific target)
    if("${PLATFORM}" STREQUAL "pico")
        pico_enable_stdio_usb("${target}" 1)
        pico_enable_stdio_uart("${target}" 0)
        pico_add_extra_outputs("${target}")
    endif()
endfunction()

# Stop Pico SDK from trying to fetch/build picotool on this machine
set(PICO_SDK_FETCH_PICOTOOL OFF)

project(infinite-feeder C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(lib)

if("${PLATFORM}" STREQUAL "pico")
    add_executable(firmware firmware.cpp
    )

    target_link_libraries(firmware
        lib-pi
        lib-pi-threads
    )

    pico_specific(firmware)
endif()
