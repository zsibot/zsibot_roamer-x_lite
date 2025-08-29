#! /bin/bash

set -e

source /opt/ros/humble/setup.bash

# 检查是否传入了参数
if [ $# -eq 0 ]; then
  echo "Usage: $0 {all deploy}"
  echo "./build.sh all             [this command will build all project which need gazebo env.]"
  echo "./build.sh all debug       [this command will build nav2 packages in debug mode]"
  exit 1
fi

case $1 in
all)
  echo "[jszr shell log] => will compile all project..."
  echo "[jszr shell log] => "

  colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON $CMAKE_DEBUG_ARGS --packages-select \
    mc_sdk_msg \
    robots_dog_msgs \
    robot_slam \
    navigo_behavior_tree \
    navigo_behaviors \
    navigo_bt_navigator \
    navigo_collision_monitor \
    navigo_core \
    navigo_costmap_2d \
    navigo_map_server \
    navigo_mppi_controller \
    navigo_path_controller \
    navigo_navfn_planner \
    navigo_path_planner \
    navigo_util \
    navigo_velocity_optimizer \
    navigo_waypoint_follower \
    fast_gicp \
    ndt_omp \
    localization \
    robot_navigo  --parallel-workers 8
  echo "[zsibot shell log] => "
  echo "[zsibot shell log] => OK"

esac
