from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Declare use_sim_time argument
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # Build MoveIt configuration
    moveit_config = MoveItConfigsBuilder("ros2_control.xacro", package_name="kinova_moveit_config").to_moveit_configs()
    
    ld = LaunchDescription()

    # Declare the 'use_sim_time' argument
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="true"))

    # 1. Static virtual joint TFs (if exists)
#  ld.add_action(
#    IncludeLaunchDescription(
#            PythonLaunchDescriptionSource([
#                PathJoinSubstitution([
#                    FindPackageShare("so_arm_moveit_config"),
#                    "launch",
#                    "static_virtual_joint_tfs.launch.py"
#                ])
#            ]),
#            launch_arguments={"use_sim_time": use_sim_time}.items()
#        )
# ros2 param set /static_transform_publisher0 use_sim_time True

    
    # 2. Robot State Publisher
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("kinova_moveit_config"),
                    "launch",
                    "rsp.launch.py"
                ])
            ]),
            launch_arguments={"use_sim_time": use_sim_time}.items()
        )
    )
    
    # 3. Move Group
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("kinova_moveit_config"),
                    "launch",
                    "move_group.launch.py"
                ])
            ]),
            launch_arguments={"use_sim_time": use_sim_time}.items()
        )
    )
    
    # 4. ros2_control_node (MoveIt-style)
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
                {'use_sim_time': use_sim_time},
            ],
            output="screen"
        )
    )
    
    # 5. Spawn Controllers
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("kinova_moveit_config"),
                    "launch",
                    "spawn_controllers.launch.py"
                ])
            ]),
            launch_arguments={"use_sim_time": use_sim_time}.items()
        )
    )
    
    return ld
