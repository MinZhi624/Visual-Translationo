from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 定义参数
    Test = Node(package="armor_plate_identification",executable="Test")
    return LaunchDescription([
        Test
    ])
