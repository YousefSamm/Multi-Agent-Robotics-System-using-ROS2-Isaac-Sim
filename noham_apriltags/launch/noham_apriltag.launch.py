from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    apriltag_ros_share_dir = get_package_share_directory('apriltag_ros')
    tags_36h11_yaml_file = os.path.join(apriltag_ros_share_dir, 'cfg', 'tags_36h11.yaml')
    launch_description = LaunchDescription([
    Node(

            package='apriltag_ros',

            executable='apriltag_node',

            name='apriltag_node',

            output='screen',

            remappings=[

                ('image_rect', '/rgb'),

                ('camera_info', '/camera_info')

            ],

            parameters=[

                {'params_file': tags_36h11_yaml_file}

            ]

        ),

        Node(

            package='noham_apriltags',

            executable='apriltag_detector',

            output='screen',

        )

    ])



    return launch_description
