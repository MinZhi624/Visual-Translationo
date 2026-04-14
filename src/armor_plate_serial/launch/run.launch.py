from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    serial_node_params_file = os.path.join(
        get_package_share_directory('armor_plate_serial'),
        'config',
        'params.yaml'
    )
    serial_node = Node(
        package='armor_plate_serial',
        executable='serial_node',
        name='armor_plate_serial_node',
        parameters=[serial_node_params_file]
    )
    return LaunchDescription([
        serial_node
    ])
