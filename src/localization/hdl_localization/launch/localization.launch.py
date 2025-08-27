import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 获取配置文件路径
    localization_dir = get_package_share_directory('localization')
    config_file = os.path.join(localization_dir, 'config', 'config.yaml')
    
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        
        # 启动全局地图服务器节点
        Node(
            package='localization',
            executable='hdl_localization_map_server',
            name='hdl_globalmap_server',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('/velodyne_points', '/livox/lidar')
            ]
        ),
        
        # 启动定位节点
        Node(
            package='localization',
            executable='hdl_localization_node',
            name='hdl_localization',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('/velodyne_points', '/livox/lidar'),
                ('/gpsimu_driver/imu_data', '/livox/imu')
            ]
        ),
        
        # 静态TF发布器
        Node(
            name='lidar_tf',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.0', '0', '0', '0', '1', 'odom', 'velodyne']
        )
    ])


