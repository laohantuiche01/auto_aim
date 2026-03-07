cmake_minimum_required(VERSION 3.16)

set(OpenVINO_DIR "/usr/local/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_subdirectory(planner/tinympc)

add_library(auto_aim STATIC
    armor.cpp
    classifier.cpp
    detector.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    shooter.cpp
    yolo.cpp
    yolos/yolov5.cpp
    yolos/yolov8.cpp
    yolos/yolo11.cpp
    multithread/commandgener.cpp
    multithread/mt_detector.cpp
    planner/planner.cpp
)

# Include directories
target_include_directories(auto_aim PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../tools
    ${CMAKE_CURRENT_SOURCE_DIR}/../io
    ${OpenCV_INCLUDE_DIRS}
)

# 添加 rclcpp 依赖（因为算法层现在使用 rclcpp::Node）
ament_target_dependencies(auto_aim rclcpp)

target_link_libraries(auto_aim io openvino::runtime tinympcstatic)

# 使用 ament 包装
ament_export_dependencies(rclcpp)