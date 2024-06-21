cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(chu_pico C CXX ASM)

set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_subdirectory(src)
