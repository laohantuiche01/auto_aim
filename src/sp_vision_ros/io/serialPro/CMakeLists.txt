cmake_minimum_required(VERSION 3.20)
project(serialPro)

set(CMAKE_SKIP_CLEAN_DEPENDS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/obj)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/obj)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(
        thirdparty/serialib
)

add_library(serialPro
        src/RobotComm.cpp
)

target_include_directories(serialPro PUBLIC
        include
        include/serialPro
        ${CMAKE_CURRENT_SOURCE_DIR}/msg_serializer
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/serialib
)

target_link_libraries(serialPro PUBLIC serialib)