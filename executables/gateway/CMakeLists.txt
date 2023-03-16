cmake_minimum_required(VERSION 3.12)

# rest of your project
add_executable(gateway
  gateway.c
)

option(USE_DEBUG "Enter debug mode" OFF)

if (USE_DEBUG MATCHES ON)
  add_definitions(-DDEBUG)
endif()

# pull in common dependencies

add_library(algobsec STATIC IMPORTED)

target_link_libraries(gateway
    pico_lorawan
    pico_stdlib
    hardware_i2c
    pico_stdio_usb
    hardware_rtc
    hardware_sleep
    pico_runtime  
)

# enable usb output, disable uart output
pico_enable_stdio_usb(gateway 1)
pico_enable_stdio_uart(gateway 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(gateway)
