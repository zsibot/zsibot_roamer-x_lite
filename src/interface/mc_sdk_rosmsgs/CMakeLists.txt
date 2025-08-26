cmake_minimum_required(VERSION 3.10)
project(mc_sdk_msg)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HighLevelCmd.msg"
  "msg/HighLevelRobotState.msg"
  "msg/LowLevelCmd.msg"
  "msg/LowLevelRobotState.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

install(
        DIRECTORY msg
        DESTINATION share/${PROJECT_NAME}
)
ament_package()