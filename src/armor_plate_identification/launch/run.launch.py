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
    armor_plate_identification_node = Node(
        package="armor_plate_identification",
        executable="ArmorPlateIdentifcation",
        parameters=[params_file]
    )
    return LaunchDescription([
        armor_plate_identification_node
    ])
