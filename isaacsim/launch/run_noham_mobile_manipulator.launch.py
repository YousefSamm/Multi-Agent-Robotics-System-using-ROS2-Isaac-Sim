# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Get the package source directory
    isaacsim_src_dir = os.path.join(
        os.path.dirname(os.path.dirname(__file__)), 'config'
    )
    
    # Path to the sim.py file
    sim_script = os.path.join(isaacsim_src_dir, 'mobile_manipulator_sim.py')
    
    # Launch Isaac Sim with the sim script
    isaacsim_node = Node(
        package='isaacsim',
        executable='run_isaacsim.py',
        name='isaacsim_jaco_husky',
        output="screen",
        parameters=[{
            'standalone': sim_script,
            'ros_distro': 'humble',
            'dds_type': 'fastdds'
        }],
        arguments=['--ros-args', '--log-level', 'isaacsim_jaco_husky:=error'],
    )
    
    return LaunchDescription([
        isaacsim_node
    ])
