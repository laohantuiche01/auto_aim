cmake_minimum_required(VERSION 3.8)
project(sp_vision_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 自动收集消息文件
get_filename_component(BASE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)

set(FORMATTED_FILES "")
file(GLOB MSG_FILES "msg/*.msg")
foreach(msg_file ${MSG_FILES})
  file(RELATIVE_PATH REL_PATH "${BASE_PATH}" "${msg_file}")
  list(APPEND FORMATTED_FILES "${BASE_PATH}:${REL_PATH}")
endforeach()

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  ${FORMATTED_FILES}
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

ament_package()
