cmake_minimum_required(VERSION 3.13)

include(lib/platform.cmake)

function(file_to_h target file output_dir)
    get_filename_component(base "${file}" NAME)
    set(output "${output_dir}/${base}.h")
    message("COMMAND: ${file} -> ${output}")
    add_custom_command(
        OUTPUT "${output}"
        COMMAND xxd -n "${base}" -i "${file}" "${output}"
        DEPENDS "${file}"
    )
    target_sources(${target}
	PUBLIC FILE_SET HEADERS
	    FILES     "${output}"
	)
endfunction()

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
        uart-channel.cpp
    )

    target_link_libraries(firmware
        lib-pi
        lib-pi-threads
    )

    pico_specific(firmware)
endif()

if ("${HAVE_NETWORKING}" STREQUAL "ON")
  add_executable(ui ui.cpp
    uart-channel.cpp
  )

  target_include_directories(ui PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
  file_to_h(ui ${CMAKE_CURRENT_LIST_DIR}/html/index.html ${CMAKE_CURRENT_BINARY_DIR})
  file_to_h(ui ${CMAKE_CURRENT_LIST_DIR}/html/config.html ${CMAKE_CURRENT_BINARY_DIR})
  file_to_h(ui ${CMAKE_CURRENT_LIST_DIR}/html/infinite-feeder.css ${CMAKE_CURRENT_BINARY_DIR})
  file_to_h(ui ${CMAKE_CURRENT_LIST_DIR}/html/favicon.ico ${CMAKE_CURRENT_BINARY_DIR})

  target_link_libraries(ui
    lib-pi
    lib-pi-net
    lib-pi-threads
  )

  pico_specific(ui)
endif()
