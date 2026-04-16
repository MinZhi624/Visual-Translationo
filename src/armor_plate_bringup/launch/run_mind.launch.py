from launch import LaunchDescription
from launch_ros.actions import Node
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
    mv_camera_params_file = os.path.join(
        get_package_share_directory('mindvision_camera'),
        'config',
        'camera_params.yaml'
    )
    mv_camera_info_url = 'package://mindvision_camera/config/camera_info.yaml'

    mv_camera_node = Node(
        package='mindvision_camera',
        executable='mindvision_camera_node',
        name='mv_camera',
        output='screen',
        emulate_tty=True,
        parameters=[mv_camera_params_file, {
            'camera_info_url': mv_camera_info_url,
        }]
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

    return LaunchDescription([
        mv_camera_node,
        identification_node,
        tracker_node,
        visualization_node,
        serial_node
    ])
