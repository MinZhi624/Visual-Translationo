from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    video_path_arg = DeclareLaunchArgument(
        name="video_path",
        default_value=get_package_share_directory("armor_plate_identification") + "/video/Blue.mp4"
    )

    identification_params_file = os.path.join(
        get_package_share_directory('armor_plate_identification'),
        'config',
        'params.yaml'
    )
    tracker_params_file = os.path.join(
        get_package_share_directory('armor_plate_tracker'),
        'config',
        'params.yaml'
    )

    test_node = Node(
        package="armor_plate_identification",
        executable="Test",
        arguments=[LaunchConfiguration("video_path")],
        parameters=[
            identification_params_file,
            # 强制覆盖：自动测试专用参数
            {"headless": True},
            {"debug_frame": True},
            {"debug_frame_count": 150},
            {"delay_time": 0},
        ]
    )

    tracker_node = Node(
        package='armor_plate_tracker',
        executable='armor_plate_tracker_node',
        name='armor_plate_tracker_node',
        parameters=[tracker_params_file]
    )

    return LaunchDescription([
        video_path_arg,
        test_node,
        tracker_node,
    ])
