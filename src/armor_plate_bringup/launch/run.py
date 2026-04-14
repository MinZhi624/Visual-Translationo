from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('armor_plate_identification'),
        'config',
        'params.yaml'
    )

    identification_node = Node(
        package='armor_plate_identification',
        executable='ArmorPlateIdentifcation',
        output='screen',
        emulate_tty=True,
        parameters=[params_file]
    )

    tracker_node = Node(
        package='armor_plate_tracker',
        executable='armor_plate_tracker_node',
        name='armor_plate_tracker_node',
        output='screen',
        emulate_tty=True,
    )

    visualization_node = Node(
        package='armor_plate_data_visualization',
        executable='data_visualization_node',
        name='data_visualization_node',
        output='screen',
        emulate_tty=True,
    )
    

    return LaunchDescription([
        identification_node,
        tracker_node,
        visualization_node,
    ])
