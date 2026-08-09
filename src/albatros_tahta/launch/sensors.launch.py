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

    mesafe_sensor_node = Node(
        package='albatros_tahta',
        executable='mesafe_sensor_node',
        name='mesafe_sensor_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    imu_node = Node(
        package='albatros_tahta',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    gps_node = Node(
        package='albatros_tahta',
        executable='gps_node',
        name='gps_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        kamera_node,
        mesafe_sensor_node,
        imu_node,
        gps_node
    ])
