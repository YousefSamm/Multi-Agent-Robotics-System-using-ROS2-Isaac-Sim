import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    
    nav2_yaml = os.path.join(get_package_share_directory('noham_localization'), 'config', 'amcl_config.yaml')
    map_file = os.path.join(get_package_share_directory('noham_map_server'), 'config', 'noham_sim_2.yaml')

    # Create the AMCL node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml, {'use_sim_time': True}]
    )

    # Create the lifecycle manager node
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'bond_timeout': 0.0},
                    {'node_names': ['map_server', 'amcl']}]
    )

    # Create the initial pose publisher using ros2 topic pub
    initial_pose_publisher = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/initialpose', 
             'geometry_msgs/msg/PoseWithCovarianceStamped', 
             ('{"header": {"frame_id": "map"}, "pose": {"pose": '
              '{"position": {"x": 0.0, "y": 0.0, "z": 0.0}, '
              '"orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}}')],
        output='screen'
    )

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename':map_file}]
        ),
            
        amcl_node,

        lifecycle_manager,

        # Publish initial pose after AMCL node starts 
        # (which will be ready to receive it)
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=amcl_node,
                on_start=[initial_pose_publisher]
            )
        )
    ])