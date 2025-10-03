from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python import get_package_share_directory


def generate_launch_description():

    nav2_yaml = os.path.join(
        get_package_share_directory('noham_path_planner'),
        'config',
        'forklift_path_planner.yaml'
    )
    controller_yaml = os.path.join(
        get_package_share_directory('noham_path_planner'),
        'config',
        'forklift_controller.yaml'
    )
    bt_navigator_yaml = os.path.join(
        get_package_share_directory('noham_path_planner'),
        'config',
        'forklift_bt_navigator.yaml'
    )
    recovery_yaml = os.path.join(
        get_package_share_directory('noham_path_planner'),
        'config',
        'forklift_recovery.yaml'
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_yaml],
        respawn=True,
        respawn_delay=2.0
    )

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='log',
        parameters=[controller_yaml],
        arguments=['--ros-args', '--log-level', 'controller_server:=warn'],
        respawn=True,
        respawn_delay=2.0
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml],
        respawn=True,
        respawn_delay=2.0
    )

    recovery_behaviors = Node(
        package='nav2_behaviors',
        name='recoveries_server',
        executable='behavior_server',
        parameters=[recovery_yaml],
        respawn=True,
        respawn_delay=2.0
    )

    smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_yaml],
        respawn=True,
        respawn_delay=2.0
    )

    cmdvel_to_ackermann_node = Node(
         package='cmdvel_to_ackermann',
         executable='cmdvel_to_ackermann.py',
         name='cmdvel_to_ackermann',
         output='screen',
         parameters=[
             {'publish_period_ms': 20},
             {'track_width': 0.407},  # Match forklift track width
             {'acceleration': 0.0},
             {'steering_velocity': 0.0}
         ]
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
            {'bond_timeout': 0.0},
            {'bond_respawn_max_duration': 0.0},
        ],
    )

    return LaunchDescription([
        planner,
        controller,
        bt_navigator,
        recovery_behaviors,
        smoother,
        life_cycle_manager,
        cmdvel_to_ackermann_node,
    ])
