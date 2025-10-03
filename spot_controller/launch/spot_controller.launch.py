from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    policy_path = os.path.join(
        get_package_share_directory('spot_controller'),
        'policy/my_spot_policy.pt'  # Adjust the policy path as needed
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'policy_path',
            default_value=policy_path,
            description='Path to the Spot controller policy file.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation time if set to True.'
        ),
        Node(
            package='spot_controller',
            executable='the_spot_controller.py',
            name='spot_controller',
            output='screen',
            parameters=[{
                'policy_path': LaunchConfiguration('policy_path'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )
    ])