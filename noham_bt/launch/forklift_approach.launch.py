#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='noham_bt',
            executable='forklift_approach',
            name='forklift_approach_node',
            output='screen',
            parameters=[{
                # You can add parameters here if needed
            }],
            remappings=[
                # Remap topics if they have different names in your system
                ('/tag_detections', '/tag_detections'),
                ('/camera_info', '/camera_info'),
                ('/cmd_vel', '/cmd_vel'),
            ]
        )
    ])
