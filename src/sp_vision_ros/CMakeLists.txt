cmake_minimum_required(VERSION 3.16)
project(sp_vision_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(OpenCV REQUIRED)

# Find custom messages
find_package(sp_vision_msgs REQUIRED)

# Find third-party libraries
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# 设置 OpenVINO 路径
set(OpenVINO_DIR "/usr/local/runtime/cmake")
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

# Include directories
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 添加子目录 (这些子目录有自己的 CMakeLists.txt)
# 注意顺序：tools 必须在 io 之前，因为 io 依赖 tools
add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(auto_aim)

# ===== 节点可执行文件 =====

# Camera Node
add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node tools io ${OpenCV_LIBS})
ament_target_dependencies(camera_node
    rclcpp
    sensor_msgs
    cv_bridge
)

# Gimbal Node
add_executable(gimbal_node src/gimbal_node.cpp)
target_link_libraries(gimbal_node tools io)
ament_target_dependencies(gimbal_node
    rclcpp
    sp_vision_msgs
    tf2_ros
    geometry_msgs
)

# Auto Aim Node (前端检测)
add_executable(auto_aim_node src/auto_aim_node.cpp)
target_link_libraries(auto_aim_node
    auto_aim
    tools
    ${OpenCV_LIBS}
)
ament_target_dependencies(auto_aim_node
    rclcpp
    sensor_msgs
    sp_vision_msgs
    cv_bridge
)

# Armor Solver Node (后端处理)
add_executable(armor_solver_node src/armor_solver_node.cpp)
target_link_libraries(armor_solver_node
    auto_aim
    tools
    ${OpenCV_LIBS}
)
ament_target_dependencies(armor_solver_node
    rclcpp
    sensor_msgs
    geometry_msgs
    sp_vision_msgs
    tf2_ros
    tf2_geometry_msgs
    tf2_eigen
)

# Install executables
install(TARGETS
    camera_node
    gimbal_node
    auto_aim_node
    armor_solver_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

# Install assets
install(DIRECTORY assets DESTINATION share/${PROJECT_NAME}/)

ament_package()
