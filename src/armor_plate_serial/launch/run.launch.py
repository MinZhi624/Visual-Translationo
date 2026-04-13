from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    serial_node = Node( package='armor_plate_serial',
        executable='serial_node',
        name='armor_plate_serial_node',
    )
    return LaunchDescription([
        serial_node
    ])
