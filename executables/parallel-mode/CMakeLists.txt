cmake_minimum_required(VERSION 3.12)

# rest of your project
add_executable(parallel-mode
  parallel_mode.c
)

option(USE_DEBUG "Enter debug mode" OFF)

if (USE_DEBUG MATCHES ON)
  add_definitions(-DDEBUG)
endif()

# pull in common dependencies

target_link_libraries(parallel-mode 
    bme68x
    bme_api
    pico_stdlib
    hardware_i2c
    pico_runtime  
)

# enable usb output, disable uart output
pico_enable_stdio_usb(parallel-mode 1)
pico_enable_stdio_uart(parallel-mode 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(parallel-mode)
