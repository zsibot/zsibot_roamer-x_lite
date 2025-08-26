import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    pkg_share_dir = get_package_share_directory('pub_tf')

    tf_type_arg = DeclareLaunchArgument(
        'tf_type', 
        default_value='localization_tf', 
        description='Use localization_tf or mc_tf or gazebo_tf or mujoco_tf'
    )

    platform_arg = DeclareLaunchArgument(
        'platform',
        default_value='',  
        description='Platform configuration'
    )

    mc_controller_type_arg = DeclareLaunchArgument(
        'mc_controller_type',
        default_value='',  
        description='cm controller type configuration'
    )

    parameter_config = PathJoinSubstitution([
        pkg_share_dir,
        'config',
        'config.yaml'
    ])
    
    return LaunchDescription([
        tf_type_arg,
        platform_arg,
        mc_controller_type_arg,
        Node(
            package='pub_tf',
            executable="tf_manager",
            parameters=[
                parameter_config,
                {
                "tf_type": LaunchConfiguration("tf_type"),
                "platform": LaunchConfiguration("platform"),
                "mc_controller_type": LaunchConfiguration("mc_controller_type")
                }
            ],
            output='screen'  
        )
    ])
