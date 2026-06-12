import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config_file = os.path.join(
        get_package_share_directory("armor_plate_serial"),
        "config",
        "serial_latency_test.yaml",
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config_file,
        description="串口延迟测试参数文件",
    )

    latency_test_node = Node(
        package="armor_plate_serial",
        executable="serial_latency_test",
        name="serial_latency_test",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("config_file")],
    )

    return LaunchDescription([
        config_file_arg,
        latency_test_node,
    ])
