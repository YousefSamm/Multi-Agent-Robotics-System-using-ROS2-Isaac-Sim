#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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
    
    # AprilTag detection node
    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        remappings=[
            ('image', 'rgb'),  # front_stereo_camera/left/image_rect_color
            ('camera_info', 'camera_info')  # front_stereo_camera/left/camera_info
        ],
        parameters=[{'size': 0.1,
                     'max_tags': 64,
                     'tile_size': 4}]
    )

    # AprilTag container
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
    
    # Mobile manipulator behavior tree node
    mobile_manipulator_bt_node = Node(
        package='noham_bt',
        executable='mobile_manipulator_bt',
        name='mobile_manipulator_behavior_tree',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ]
    )
    
    # Launch all nodes together - they will start in parallel but with proper dependencies
    return LaunchDescription([
        declare_use_sim_time,
        apriltag_container,
        manipulator_control_node,
        mobile_manipulator_bt_node,
    ])
