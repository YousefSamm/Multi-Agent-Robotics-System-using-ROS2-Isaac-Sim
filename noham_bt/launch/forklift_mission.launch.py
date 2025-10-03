#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Forklift Behavior Tree Node
    forklift_bt_node = Node(
        package='noham_bt',
        executable='forklift_bt',
        name='forklift_behavior_tree',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[],
        arguments=[]
    )
    
    # Noham Actions Node
    noham_actions_node = Node(
        package='noham_bt',
        executable='noham_actions',
        name='noham_actions',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[],
        arguments=[]
    )
    
    # Create the AprilTag node
    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        remappings=[
            ('image', 'rgb'),  # front_stereo_camera/left/image_rect_color
            ('camera_info', 'camera_info')  # camera_info topic
        ],
        parameters=[{'size': 0.1,
                     'max_tags': 64,
                     'tile_size': 4}]
    )

    # Create the AprilTag container
    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            apriltag_node,
        ],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        forklift_bt_node,
        noham_actions_node,
        apriltag_container
    ])
