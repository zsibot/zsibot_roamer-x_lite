#!/bin/bash

# 启动全局地图服务器节点
ros2 run localization hdl_localization_map_server &
MAP_SERVER_PID=$!

# 等待一下确保地图服务器启动
sleep 2

# 启动定位节点
ros2 run localization hdl_localization_node &
LOCALIZATION_PID=$!

# 启动静态TF发布器
ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0 0 0 1 odom velodyne &
TF_PID=$!

echo "所有节点已启动"
echo "地图服务器 PID: $MAP_SERVER_PID"
echo "定位节点 PID: $LOCALIZATION_PID"
echo "TF发布器 PID: $TF_PID"

# 等待用户中断
trap "echo '正在停止所有节点...'; kill $MAP_SERVER_PID $LOCALIZATION_PID $TF_PID; exit" INT

# 保持脚本运行
wait 