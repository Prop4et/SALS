cmake_minimum_required(VERSION 3.12)

# rest of your project
add_executable(class-c
  class_c.c
  hw_config.c
)

option(USE_DEBUG "Enter debug mode" OFF)

if (USE_DEBUG MATCHES ON)
  add_definitions(-DDEBUG)
endif()

# pull in common dependencies
SET_TARGET_PROPERTIES(bsec PROPERTIES LINKER_LANGUAGE C)

add_library(algobsec STATIC IMPORTED)
set_property(TARGET algobsec PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../lib/bme/bsec/libalgobsec.a)


target_link_libraries(class-c 
    pico_lorawan
    bme68x
    bme_api
    bsec
    pico_stdlib
    hardware_i2c
    algobsec
    FatFs_SPI
    pico_stdio_usb
    hardware_rtc
    hardware_sleep
    pico_runtime  
)

# enable usb output, disable uart output
pico_enable_stdio_usb(class-c 0)
pico_enable_stdio_uart(class-c 1)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(class-c)
