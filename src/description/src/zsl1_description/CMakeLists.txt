cmake_minimum_required(VERSION 3.8)
project(zsl1_description)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(robots_dog_msgs REQUIRED)

include_directories(
        ${geometry_msgs_INCLUDE_DIRS}
        ${CMAKE_BINARY_DIR}/rosidl_generator_cpp
)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

install(
        DIRECTORY urdf meshes
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
