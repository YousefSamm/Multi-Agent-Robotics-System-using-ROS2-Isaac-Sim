#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Set MoveIt configuration
    moveit_config = MoveItConfigsBuilder("kinova").to_moveit_configs()
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Manipulator control node
    manipulator_control_node = Node(
        package='noham_bt',
        executable='manipulator_control',
        name='manipulator_control_node',
        output='screen',
        parameters=[
            moveit_config.robot_description,  # Load URDF
            moveit_config.robot_description_semantic,  # Load SRDF
            moveit_config.robot_description_kinematics,  # Load kinematics.yaml
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/isaac_joint_states', '/isaac_joint_states'),
            ('/isaac_joint_command', '/isaac_joint_command'),
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        manipulator_control_node,
    ])
