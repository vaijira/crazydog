"""
sim.launch.py  —  Gazebo Harmonic simulation launch for the Unitree Go2.

What this does:
  1. Xacro → URDF (robot_description parameter)
  2. Starts robot_state_publisher   → broadcasts TF + /joint_states shape
  3. Starts Gazebo (gz sim)         → physics engine
  4. Spawns the robot               → gz service call
  5. Loads ros2_control controllers → joint_state_broadcaster + go2_joint_controller

Usage:
  ros2 launch go2_bringup sim.launch.py [world:=<path>] [use_rviz:=true]
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ------------------------------------------------------------------ #
    # Package paths
    # ------------------------------------------------------------------ #
    go2_description_pkg = get_package_share_directory("go2_description")
    go2_bringup_pkg     = get_package_share_directory("go2_bringup")

    # ------------------------------------------------------------------ #
    # Launch arguments
    # ------------------------------------------------------------------ #
    declared_args = [
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(go2_bringup_pkg, "worlds", "empty.sdf"),
            description="Path to the Gazebo world SDF file",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz2 alongside Gazebo",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use /clock from Gazebo",
        ),
        DeclareLaunchArgument(
            "gz_args",
            default_value="-r -v 1",
            description="Extra args to pass to gz sim (e.g. headless: '-s -r')",
        ),
    ]

    world      = LaunchConfiguration("world")
    use_rviz   = LaunchConfiguration("use_rviz")
    use_sim    = LaunchConfiguration("use_sim_time")
    gz_args    = LaunchConfiguration("gz_args")

    # ------------------------------------------------------------------ #
    # Robot description (XACRO → URDF string)
    # ------------------------------------------------------------------ #
    xacro_file = os.path.join(go2_description_pkg, "urdf", "go2.urdf.xacro")
    robot_description_content = Command(
        [FindExecutable(name="xacro"), " ", xacro_file]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ------------------------------------------------------------------ #
    # robot_state_publisher
    # ------------------------------------------------------------------ #
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim}],
    )

    # ------------------------------------------------------------------ #
    # Gazebo Harmonic (gz sim)
    # ------------------------------------------------------------------ #
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": [gz_args, " ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # ------------------------------------------------------------------ #
    # Spawn robot into Gazebo
    # ------------------------------------------------------------------ #
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "go2",
            "-world", "go2_empty",
            "-topic", "robot_description",
            "-z", "0.5",          # spawn 0.5 m above ground
        ],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Bridge /clock from Gazebo to ROS 2
    # ------------------------------------------------------------------ #
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Load controllers (after spawn)
    # ------------------------------------------------------------------ #
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_joint_controller = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "go2_joint_controller",
        ],
        output="screen",
    )

    # Chain: spawn → load JSB → load joint controller
    load_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    load_jc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_joint_controller],
        )
    )
    # ------------------------------------------------------------------ #
    # High-level Control Nodes
    # ------------------------------------------------------------------ #
    go2_behavior = Node(
        package="go2_behavior",
        executable="go2_behavior",
        output="screen",
    )

    go2_gait = Node(
        package="go2_gait",
        executable="go2_gait",
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Optional RViz2
    # ------------------------------------------------------------------ #
    rviz_config = os.path.join(go2_bringup_pkg, "config", "go2_rviz.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        condition=__import__("launch.conditions", fromlist=["IfCondition"]).IfCondition(use_rviz),
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Assemble
    # ------------------------------------------------------------------ #
    return LaunchDescription(
        declared_args
        + [
            robot_state_publisher,
            gz_sim,
            spawn_robot,
            gz_bridge,
            load_jsb_after_spawn,
            load_jc_after_jsb,
            go2_behavior,
            go2_gait,
            rviz,
        ]
    )
