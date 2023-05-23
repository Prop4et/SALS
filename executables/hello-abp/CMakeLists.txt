cmake_minimum_required(VERSION 3.12)

# rest of your project
add_executable(hello-abp
  hello_abp.c
)

option(USE_DEBUG "Enter debug mode" OFF)

if (USE_DEBUG MATCHES ON)
  add_definitions(-DDEBUG)
endif()

# pull in common dependencies


target_link_libraries(hello-abp 
    pico_lorawan
    pico_stdlib
    hardware_i2c
    pico_stdio_usb
    pico_runtime  
)

# enable usb output, disable uart output
pico_enable_stdio_usb(hello-abp 1)
pico_enable_stdio_uart(hello-abp 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(hello-abp)
