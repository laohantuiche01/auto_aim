cmake_minimum_required(VERSION 3.20)
project(serialPro)

set(CMAKE_SKIP_CLEAN_DEPENDS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/obj)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/obj)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

include_directories(include msg_serializer thirdparty/serialib)

add_subdirectory(thirdparty/serialib)

add_library(serialPro STATIC
        src/msg_serializer.cpp
        src/robot_serial.cpp
        src/referee_serial.cpp
)