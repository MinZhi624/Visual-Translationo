from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    # 定义参数
    video_default_path = get_package_share_directory("armor_plate_identification") + "/video/Blue.mp4"
    video_blue_fast = get_package_share_directory("armor_plate_identification") + "/video/BlueFast.mp4"
    video_blue_fast_rotated_and_move = get_package_share_directory("armor_plate_identification") + "video/BlueFastRotatedAndMove.mp4"
    video_blue_slow = get_package_share_directory("armor_plate_identification") + "/video/BlueSlow.mp4"

    video_path_arg = DeclareLaunchArgument(
        name="video_path",
        default_value= video_default_path 
        # default_value = video_blue_fast
        # default_value = video_blue_fast_rotated_and_move
        # default_value = video_blue_slow
    )

    params_file = os.path.join(
        get_package_share_directory('armor_plate_identification'),
        'config',
        'params.yaml'
    )

    # 定义节点
    test_node = Node(
        package="armor_plate_identification",
        executable="Test",
        arguments=[LaunchConfiguration("video_path")],
        parameters=[params_file]
    )

    return LaunchDescription([
        video_path_arg,
        test_node,
    ])
