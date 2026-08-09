from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Sensör ve Donanım Katmanı Node'ları
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

    # Görüntü İşleme & Algılama Katmanı Node'ları
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

    # Haritalama, Karar & Kontrol Katmanı Node'ları
    costmap_node = Node(
        package='albatros_tahta',
        executable='costmap_node',
        name='costmap_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    karar_node = Node(
        package='albatros_tahta',
        executable='karar_node',
        name='karar_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    state_node = Node(
        package='albatros_tahta',
        executable='state_node',
        name='state_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    mission_node = Node(
        package='albatros_tahta',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    kontrol_node = Node(
        package='albatros_tahta',
        executable='kontrol_node',
        name='kontrol_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        kamera_node,
        mesafe_sensor_node,
        imu_node,
        gps_node,
        yolo_node,
        yolo_mesafe_node,
        duba_fusion_node,
        costmap_node,
        karar_node,
        state_node,
        mission_node,
        kontrol_node
    ])
