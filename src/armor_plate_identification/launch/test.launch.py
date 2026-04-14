from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    # 定义参数
    video_default_path = get_package_share_directory("armor_plate_identification") + "/video/Blue.mp4"
    video_true_path1 = get_package_share_directory("armor_plate_identification") + "/video/1.avi"
    video_true_path2 = get_package_share_directory("armor_plate_identification") + "/video/2.avi"
    
    video_path_arg = DeclareLaunchArgument(
        name="video_path",
        default_value= video_default_path 
        # default_value= video_true_path1 
        # default_value= video_true_path2 
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
