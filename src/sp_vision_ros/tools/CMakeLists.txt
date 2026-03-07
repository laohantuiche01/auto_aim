cmake_minimum_required(VERSION 3.16)

find_package(fmt REQUIRED)

add_library(tools STATIC
    exiter.cpp
    extended_kalman_filter.cpp
    ransac_sine_fitter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    trajectory.cpp
    recorder.cpp
    logger.cpp
    pid.cpp
    crc.cpp
)

# Include directories
target_include_directories(tools PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# Link libraries
target_link_libraries(tools PUBLIC ${OpenCV_LIBS} fmt::fmt)