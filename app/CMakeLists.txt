cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(philly)

enable_language(C ASM)

add_subdirectory(drivers/i2s)

target_sources(app PRIVATE
    src/main.c
)
