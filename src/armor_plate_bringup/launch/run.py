from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    identification_node = Node(
        package='armor_plate_identification',
        executable='ArmorPlateIdentifcation',
        name='armor_plate_identification_node',
        output='screen',
        emulate_tty=True,
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
