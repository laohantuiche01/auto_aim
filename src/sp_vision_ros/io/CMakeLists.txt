cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_subdirectory(serial)

# 创建目标 io
add_library(io STATIC
    hikrobot/hikrobot.cpp
    mindvision/mindvision.cpp
    usbcamera/usbcamera.cpp
    camera.cpp
    cboard.cpp
    dm_imu/dm_imu.cpp
    gimbal/gimbal.cpp
)

# hikrobot
target_include_directories(io PUBLIC hikrobot/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC hikrobot/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

# mindvision
target_include_directories(io PUBLIC mindvision/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC mindvision/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC mindvision/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(io MvCameraControl MVSDK usb-1.0 yaml-cpp serial fmt::fmt tools ${OpenCV_LIBS})

# Include directories for project headers
target_include_directories(io PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../tools
    ${OpenCV_INCLUDE_DIRS}
)

# 添加 rclcpp 依赖（因为 io 层现在使用 rclcpp::Node）
ament_target_dependencies(io rclcpp)
ament_export_dependencies(rclcpp)

################## 检查 ROS 环境 ##################
# 使用 DEFINED 来避免部分包存在导致的错误
if(DEFINED ENV{AMENT_PREFIX_PATH} AND EXISTS "$ENV{AMENT_PREFIX_PATH}")
    find_package(ament_cmake QUIET)
    find_package(rclcpp QUIET)
    find_package(std_msgs QUIET)
    find_package(rosidl_typesupport_cpp QUIET)
    find_package(sp_msgs QUIET)

    # 只有所有包都找到才启用 ROS2
    if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND rosidl_typesupport_cpp_FOUND AND sp_msgs_FOUND)
        message(STATUS "ROS2 environment found, compiling ROS2-related code.")

        # 添加 ROS2 相关的文件
        target_sources(io PRIVATE ros2/publish2nav.cpp ros2/ros2.cpp ros2/subscribe2nav.cpp)

        # 设置 ROS2 依赖
        include_directories(${std_msgs_INCLUDE_DIRS})
        ament_target_dependencies(io rclcpp std_msgs sp_msgs)
    else()
        message(WARNING "ROS2 packages incomplete, skipping ROS2 specific code.")
    endif()
else()
    message(STATUS "ROS2 environment not detected, skipping ROS2 specific code.")
endif()
