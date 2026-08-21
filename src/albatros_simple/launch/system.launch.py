#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='albatros_simple',
            executable='camera_node',
            name='camera_node',
            output='screen',
        ),
        Node(
            package='albatros_simple',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
        ),
        Node(
            package='albatros_simple',
            executable='mission_node',
            name='mission_node',
            output='screen',
        ),
    ])
