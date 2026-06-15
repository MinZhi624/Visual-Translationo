from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('armor_plate_planner'),
        'config',
        'params.yaml'
    )

    planner_node = Node(
        package='armor_plate_planner',
        executable='armor_plate_planner_node',
        name='armor_plate_planner_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        planner_node,
    ])
