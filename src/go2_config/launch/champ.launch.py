import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    go2_config_share = get_package_share_directory('go2_config')
    go2_description_share = get_package_share_directory('go2_description')
    champ_bringup_share = get_package_share_directory('champ_bringup')

    description_path = os.path.join(go2_description_share, 'urdf', 'go2.urdf.xacro')
    joints_config_path = os.path.join(go2_config_share, 'config', 'joints.yaml')
    links_config_path = os.path.join(go2_config_share, 'config', 'links.yaml')
    gait_config_path = os.path.join(go2_config_share, 'config', 'gait.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(champ_bringup_share, 'launch', 'bringup.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'description_path': description_path,
                'joints_map_path': joints_config_path,
                'links_map_path': links_config_path,
                'gait_config_path': gait_config_path,
                'robot_name': '/',
                'base_link_frame': 'base_link',
                'gazebo': 'true',
                'joint_controller_topic': 'go2_joint_controller/joint_trajectory',
                'publish_joint_states': 'false',
                'publish_joint_control': 'true',
                'publish_foot_contacts': 'true',
                'publish_odom_tf': 'true',
            }.items()
        )
    ])
