cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})


find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})


project(philly)

enable_language(C ASM)

#add_subdirectory(drivers/i2s)

target_sources(app PRIVATE
src/main.c
)

include(drivers/i2s/i2s_drivers.cmake)