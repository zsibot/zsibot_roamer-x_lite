#! /bin/bash

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 方案1: 智能检测工作空间路径
find_workspace() {
    local current_dir="$1"
    
    # 如果在工作空间根目录运行
    if [ -f "$current_dir/install/setup.bash" ]; then
        echo "$current_dir"
        return 0
    fi
    
    # 如果在install目录中运行
    if [[ "$current_dir" == */install/* ]]; then
        local workspace_dir="$(dirname "$(dirname "$(dirname "$current_dir")")")"
        if [ -f "$workspace_dir/install/setup.bash" ]; then
            echo "$workspace_dir"
            return 0
        fi
    fi
    
    # 如果在src目录中运行
    if [[ "$current_dir" == */src/* ]]; then
        local workspace_dir="$(dirname "$(dirname "$current_dir")")"
        if [ -f "$workspace_dir/install/setup.bash" ]; then
            echo "$workspace_dir"
            return 0
        fi
    fi
    
    return 1
}

# 尝试从当前目录开始查找工作空间
WORKSPACE_DIR=""
if find_workspace "$(pwd)"; then
    WORKSPACE_DIR="$(find_workspace "$(pwd)")"
    echo "从当前目录检测到工作空间: $WORKSPACE_DIR"
else
    # 方案2: 回退到脚本所在目录的两层上级目录
    WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
    echo "使用脚本回退路径: $WORKSPACE_DIR"
    
    # 验证路径是否正确
    if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        echo "错误: 无法找到工作空间，请确保脚本位于正确位置"
        echo "脚本路径: $SCRIPT_DIR"
        echo "尝试的工作空间路径: $WORKSPACE_DIR"
        exit 1
    fi
fi

echo "使用工作空间路径: $WORKSPACE_DIR"

# 检查ROS2环境
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "错误: 未找到ROS2 Humble环境，请确保已安装ROS2 Humble"
    exit 1
fi

# 检查工作空间是否已构建
if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    echo "错误: 工作空间未构建，请先运行 'colcon build'"
    exit 1
fi

# source工作空间
echo "正在加载ROS2环境..."
source /opt/ros/humble/setup.bash

echo "正在加载工作空间环境..."
# source /home/ubuntu/localization/install/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

echo "启动定位节点..."
ros2 launch localization simple_localization.launch.py

