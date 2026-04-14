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

    # 定义节点
    test_node = Node(
        package="armor_plate_identification",
        executable="Test",
        arguments=[LaunchConfiguration("video_path")],
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

    serial_node = Node( package='armor_plate_serial',
        executable='serial_node',
        name='armor_plate_serial_node',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        # 定义参数
        video_path_arg,
        # 启动节点
        test_node, tracker_node, visualization_node, serial_node
    ])
