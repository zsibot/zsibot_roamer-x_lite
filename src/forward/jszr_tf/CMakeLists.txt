cmake_minimum_required(VERSION 3.22.1)
project(pub_tf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

add_definitions(-DLOCALIZATION_DEBUG)
get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
add_definitions(-DMAP_DIR="$ENV{HOME}/.jszr/map")

if(BUILD_DEBUG_PACKAGES)
  message("Building in DEBUG mode for pub_tf")
  SET(CMAKE_CXX_FLAGS "-g -O0")
  SET(CMAKE_BUILD_TYPE "Debug")  
  SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
  SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
else()
  message("Building in RELEASE mode for pub_tf")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robots_dog_msgs REQUIRED)
find_package(mc_sdk_msg REQUIRED)
find_package(zsibot_common REQUIRED)

include_directories(include)

add_executable(tf_manager  
               src/tf_manager.cpp
               src/base_tf.cpp
               src/localization_tf.cpp 
               src/gazebo_tf.cpp
               src/mujoco_tf.cpp
               src/mc_tf.cpp
               src/highlevel_connector.cpp
        )
ament_target_dependencies(tf_manager
        rclcpp
        tf2_ros
        nav_msgs
        geometry_msgs
        robots_dog_msgs
        mc_sdk_msg
)

target_link_libraries(
        tf_manager
        zsibot_common::zsibot_base
        zsibot_common::zsibot_rmw_ecal
        zsibot_common::robot_sdk
)

# add_executable(localization_tf src/localization_tf.cpp )
# ament_target_dependencies(localization_tf
#         rclcpp
#         tf2_ros
#         nav_msgs
#         geometry_msgs
#         robots_dog_msgs
# )

# add_executable(gazebo_tf src/gazebo_tf.cpp)
# ament_target_dependencies(gazebo_tf
#         rclcpp
#         tf2_ros
#         nav_msgs
#         geometry_msgs
# )

# add_executable(mc_tf src/mc_tf.cpp src/highlevel_connector.cpp)
# ament_target_dependencies(mc_tf
#         rclcpp
#         tf2_ros
#         nav_msgs
#         geometry_msgs
#         robots_dog_msgs
# )

# install(
#         TARGETS localization_tf gazebo_tf mc_tf
#         DESTINATION lib/${PROJECT_NAME}
# )

install(
        TARGETS tf_manager
        DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
