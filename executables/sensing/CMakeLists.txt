cmake_minimum_required(VERSION 3.12)

# rest of your project
add_executable(sensing
  sensing.c
)

option(USE_DEBUG "Enter debug mode" OFF)

if (USE_DEBUG MATCHES ON)
  add_definitions(-DDEBUG)
endif()

# pull in common dependencies

add_library(algobsec STATIC IMPORTED)
set_property(TARGET algobsec PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../lib/bme/bsec/libalgobsec.a)
target_link_libraries(sensing 
    bme68x
    bme_api
    bsec
    pico_stdlib
    littlefs-lib
    hardware_i2c
    algobsec
    pico_runtime  
)

# enable usb output, disable uart output
pico_enable_stdio_usb(sensing 1)
pico_enable_stdio_uart(sensing 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(sensing)
