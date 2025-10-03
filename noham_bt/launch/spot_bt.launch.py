#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    room1_pose_arg = DeclareLaunchArgument(
        'room1_pose',
        default_value='14.1706,-6.96892,0.0,0.0,0.0,0.0',
        description='Room 1 pose (x,y,z,roll,pitch,yaw)'
    )
    
    room2_pose_arg = DeclareLaunchArgument(
        'room2_pose',
        default_value='14.2639,7.78589,0.0,0.0,0.0,-1.53049',
        description='Room 2 pose (x,y,z,roll,pitch,yaw)'
    )
    
    room3_pose_arg = DeclareLaunchArgument(
        'room3_pose',
        default_value='25.3004,0.391258,0.0,0.0,0.0,-3.13383',
        description='Room 3 pose (x,y,z,roll,pitch,yaw)'
    )
    
    # Create the Spot behavior tree node
    spot_bt_node = Node(
        package='noham_bt',
        executable='spot_bt',
        name='spot_behavior_tree',
        output='screen',
        parameters=[
            {
                'room1_pose': LaunchConfiguration('room1_pose'),
                'room2_pose': LaunchConfiguration('room2_pose'),
                'room3_pose': LaunchConfiguration('room3_pose')
            }
        ]
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
        room1_pose_arg,
        room2_pose_arg,
        room3_pose_arg,
        spot_bt_node,
        apriltag_container
    ])
