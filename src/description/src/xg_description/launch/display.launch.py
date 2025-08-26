import launch_ros
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path


def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path(
        'xg_description'), 'urdf', 'xg.urdf')
    # urdf_path = os.path.join(get_package_share_path('xg_description'), 'xacro', 'robot.xacro')

    rviz_config_path = os.path.join(get_package_share_path(
        'xg_description'), 'rviz', 'xg_config.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str)  # Removed extra space

    robot_state_publiser_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path]  # Pass rviz config file
    )

    return LaunchDescription([
        robot_state_publiser_node,
        joint_state_publisher_node,
        rviz2_node
    ])
