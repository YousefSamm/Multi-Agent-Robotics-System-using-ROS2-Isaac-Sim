import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    nav2_yaml = os.path.join(
        get_package_share_directory('noham_localization'),
        'config',
        'mobile_manipulator_config.yaml'
    )
    map_file = os.path.join(
        get_package_share_directory('noham_map_server'),
        'config',
        'noham_sim_2.yaml'
    )

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                      {'yaml_filename': map_file}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml,
                       {'use_sim_time': True},
                       {'transform_tolerance': 1.0},
                       {'transform_timeout': 0.2},
                       {'initial_pose_timeout': 5.0},
                       {'pose_update_timeout': 1.0}]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                       {'autostart': True},
                       {'bond_timeout': 0.0},
                       {'node_names': ['map_server', 'amcl']}]
        )
    ])