#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'room1_pose',
            default_value='14.1888,-6.98531,0.0,0.0,0.0,1.60443',
            description='Room 1 pose (x,y,z,roll,pitch,yaw)'
        ),
        DeclareLaunchArgument(
            'room2_pose',
            default_value='25.2079,0.0553226,0.0,0.0,0.0,-3.09127',
            description='Room 2 pose (x,y,z,roll,pitch,yaw)'
        ),
        DeclareLaunchArgument(
            'room3_pose',
            default_value='14.4508,7.97975,0.0,0.0,0.0,-1.5947',
            description='Room 3 pose (x,y,z,roll,pitch,yaw)'
        ),
        
        # Launch the behavior tree node
        Node(
            package='noham_bt',
            executable='spot_bt',
            name='room_navigation_bt',
            output='screen',
            parameters=[{
                'room1_pose': LaunchConfiguration('room1_pose'),
                'room2_pose': LaunchConfiguration('room2_pose'),
                'room3_pose': LaunchConfiguration('room3_pose'),
            }]
        )
    ])
