cmake_minimum_required(VERSION 3.5)
project(ecal2ros VERSION 0.2.0)

# 添加 C++14 标准支持
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(zsibot_common REQUIRED)

# 包含头文件目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

include_directories(/opt/ros/humble/include/cv_bridge)
set(cv_bridge_LIBRARIES ${cv_bridge_LIBRARIES} /opt/ros/humble/lib/libcv_bridge.so)
add_executable(ecal2ros
  src/ecal2ros.cpp
)
target_link_libraries(ecal2ros
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
  yaml-cpp  # 使用新目标名称
  zsibot_common::zsibot_pb_msg
  zsibot_common::zsibot_base
  zsibot_common::zsibot_rmw_ecal
  zsibot_common::zsibot_param
)
ament_target_dependencies(ecal2ros
    rclcpp
    cv_bridge
)

install(TARGETS
        ecal2ros
        DESTINATION lib/${PROJECT_NAME}
)


ament_package()

