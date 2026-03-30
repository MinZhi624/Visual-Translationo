from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 定义参数
    armor_plate_identification_node = Node(package="armor_plate_identification",executable="ArmorPlateIdentifcation")
    return LaunchDescription([
        armor_plate_identification_node
    ])  