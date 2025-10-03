from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python import get_package_share_directory


def generate_launch_description():

    nav2_yaml = os.path.join(
        get_package_share_directory('noham_path_planner'),
        'config',
        'spot_path_palnner_server.yaml'
    )
    controller_yaml = os.path.join(
        get_package_share_directory('noham_path_planner'),
        'config',
        'spot_controller.yaml'
    )
    bt_navigator_yaml = os.path.join(
        get_package_share_directory('noham_path_planner'),
        'config',
        'spot_bt_navigator.yaml'
    )
    recovery_yaml = os.path.join(
        get_package_share_directory('noham_path_planner'),
        'config',
        'spot_recovery.yaml'
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_yaml]
    )

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='log',
        parameters=[controller_yaml],
        arguments=['--ros-args', '--log-level', 'controller_server:=warn']
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml]
    )

    recovery_behaviors = Node(
        package='nav2_behaviors',
        name='recoveries_server',
        executable='behavior_server',
        parameters=[recovery_yaml]
    )

    life_cycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='path_planning_lifecycle_manager',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {
                'node_names': [
                    'planner_server',
                    'controller_server',
                    'bt_navigator',
                    'recoveries_server',
                ]
            },
            # Timeout configurations to prevent state change failures
            {'bond_timeout': 0.0},
            {'bond_respawn_max_duration': 0.0},
        ],
    )

    return LaunchDescription([
        planner,
        controller,
        bt_navigator,
        recovery_behaviors,
        life_cycle_manager,
    ])
