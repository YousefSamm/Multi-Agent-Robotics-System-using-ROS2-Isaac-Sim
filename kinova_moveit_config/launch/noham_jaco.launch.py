import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Command-line arguments
    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"
    )

    moveit_config = (
        MoveItConfigsBuilder("kinova", package_name="kinova_moveit_config")
        .robot_description(file_path="config/Kinova.urdf.xacro")
        .robot_description_semantic(file_path="config/Kinova.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
         ros_arguments=[
        '--log-level', 'moveit_ros.planning_scene_monitor.planning_scene_monitor:=ERROR'
    ]
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("kinova_moveit_config"), "launch"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    # Static TF - Fixed to connect to the actual robot base
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        parameters=[{"use_sim_time": True}],
        output="log",
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0", 
                  "--qx", "0.0", "--qy", "0.0", "--qz", "0.0", "--qw", "1.0",
                  "--frame-id", "chassis_link", "--child-frame-id", "j2n6s300_link_base"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("kinova_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path, {"use_sim_time": True}],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    # Wait for ros2_control_node to be ready before spawning controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # Warehouse mongodb server
    db_config = LaunchConfiguration("db")
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
            {"use_sim_time": True}
        ],
        output="screen",
        condition=IfCondition(db_config),
    )

    # Add delays to ensure proper startup sequence
    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_arm_controller_spawner = TimerAction(
        period=3.0,
        actions=[arm_controller_spawner],
    )

    delayed_gripper_controller_spawner = TimerAction(
        period=4.0,
        actions=[gripper_controller_spawner],
    )

    return LaunchDescription(
        [
            db_arg,
            # rviz_node,  # Uncomment when needed
            #static_tf,
            #robot_state_publisher,
            ros2_control_node,
            delayed_joint_state_broadcaster_spawner,
            delayed_arm_controller_spawner,
            delayed_gripper_controller_spawner,
            run_move_group_node,
            mongodb_server_node,
        ]
    )