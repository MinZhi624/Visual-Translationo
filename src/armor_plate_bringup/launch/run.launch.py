from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    identification_node_params_file = os.path.join(
        get_package_share_directory('armor_plate_identification'),
        'config',
        'params.yaml'
    )
    tracker_node_params_file = os.path.join(
        get_package_share_directory('armor_plate_tracker'),
        'config',
        'params.yaml'
    )
    serial_node_params_file = os.path.join(
        get_package_share_directory('armor_plate_serial'),
        'config',
        'params.yaml'
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('armor_plate_bringup'),
        'rviz2',
        'rivz2.rviz'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2'
    )

    identification_node = Node(
        package='armor_plate_identification',
        executable='ArmorPlateIdentifcation',
        output='screen',
        emulate_tty=True,
        parameters=[identification_node_params_file]
    )

    tracker_node = Node(
        package='armor_plate_tracker',
        executable='armor_plate_tracker_node',
        name='armor_plate_tracker_node',
        output='screen',
        emulate_tty=True,
        parameters=[tracker_node_params_file]
    )

    visualization_node = Node(
        package='armor_plate_data_visualization',
        executable='data_visualization_node',
        name='data_visualization_node',
        output='screen',
        emulate_tty=True
    )

    serial_node = Node(
        package='armor_plate_serial',
        executable='serial_node',
        name='armor_plate_serial_node',
        output='screen',
        emulate_tty=True,
        parameters=[serial_node_params_file]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        identification_node,
        tracker_node,
        visualization_node,
        serial_node,
        rviz_node
    ])
