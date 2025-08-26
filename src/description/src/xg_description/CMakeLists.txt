cmake_minimum_required(VERSION 3.8)
project(xg_description)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(robots_dog_msgs REQUIRED)

include_directories(
  plugin/include
  ${PCL_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}/rosidl_generator_cpp
)

link_directories(${GAZEBO_LIBRARY_DIRS})



## livox_laser_simulation plugin
add_library(livox_laser_simulation_plugin SHARED 
  plugin/src/livox_laser_simulation/livox_points_plugin.cpp 
  plugin/src/livox_laser_simulation/livox_ode_multiray_shape.cpp)

ament_target_dependencies(livox_laser_simulation_plugin 
  rclcpp 
  tf2_ros 
  sensor_msgs 
  std_msgs 
  gazebo_ros 
  PCL 
  pcl_msgs 
  pcl_conversions 
  rosidl_typesupport_cpp 
  robots_dog_msgs
)

target_link_libraries(livox_laser_simulation_plugin 
  RayPlugin 
  libprotobuf.so 
  ${gazebo_ros_LIBRARIES} 
  ${rosidl_typesupport_cpp_LIBRARIES}
)

install(TARGETS livox_laser_simulation_plugin
  EXPORT export_livox_laser_simulation_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## force plugin 

add_library(unitree_foot_contact_plugin SHARED plugin/src/foot_contact_plugin.cc)
ament_target_dependencies(unitree_foot_contact_plugin rclcpp gazebo_ros std_msgs sensor_msgs geometry_msgs)
target_link_libraries(unitree_foot_contact_plugin   ${gazebo_ros_LIBRARIES})
install(TARGETS unitree_foot_contact_plugin 
  EXPORT export_unitree_foot_contact_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


add_library(unitree_draw_force_plugin SHARED plugin/src/draw_force_plugin.cc)
ament_target_dependencies(unitree_draw_force_plugin rclcpp gazebo_ros std_msgs sensor_msgs geometry_msgs)
target_link_libraries(unitree_draw_force_plugin   ${gazebo_ros_LIBRARIES})
install(TARGETS unitree_draw_force_plugin
        EXPORT export_unitree_draw_force_plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



install(DIRECTORY plugin/include/
  DESTINATION plugin/include/
)

install(
  DIRECTORY urdf meshes xacro launch rviz models scan_mode
  DESTINATION share/${PROJECT_NAME}/
)
ament_export_include_directories(plugin/include)
ament_export_libraries(livox_laser_simulation_plugin)
ament_export_libraries(unitree_draw_force_plugin)
ament_export_libraries(unitree_foot_contact_plugin)

ament_package()
