from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tracker_node = Node(
        package='armor_plate_tracker',
        executable='armor_plate_tracker_node',
        name='armor_plate_tracker_node',
    )

    return LaunchDescription([
        tracker_node,
    ])
