from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    kamera_node = Node(
        package='albatros_tahta',
        executable='kamera_node',
        name='kamera_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    yolo_node = Node(
        package='albatros_tahta',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    yolo_mesafe_node = Node(
        package='albatros_tahta',
        executable='yolo_mesafe_node',
        name='yolo_mesafe_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    duba_fusion_node = Node(
        package='albatros_tahta',
        executable='duba_fusion_node',
        name='duba_fusion_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        kamera_node,
        yolo_node,
        yolo_mesafe_node,
        duba_fusion_node
    ])
