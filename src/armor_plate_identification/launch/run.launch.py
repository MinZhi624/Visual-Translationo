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

    armor_plate_identification_node = Node(
        package="armor_plate_identification",
        executable="ArmorPlateIdentifcation",
        parameters=[params_file]
    )

    return LaunchDescription([
        mv_camera_node,
        armor_plate_identification_node
    ])
