from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # 定义参数
    video_default_path = get_package_share_directory("armor_plate_identification") + "/video/Blue.mp4"
    video_true_path = get_package_share_directory("armor_plate_identification") + "/video/1.avi"
    video_path_arg = DeclareLaunchArgument(
        name="video_path",
        default_value= video_default_path
    )
    # 定义节点
    Test = Node(
        package="armor_plate_identification",
        executable="Test",
        arguments=[LaunchConfiguration("video_path")]
    )

    return LaunchDescription([
        video_path_arg,
        Test
    ])
