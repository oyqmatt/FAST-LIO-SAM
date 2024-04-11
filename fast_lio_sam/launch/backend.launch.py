import os.path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory("fast_lio_sam")
    default_config_path = os.path.join(package_path, "config")

    rviz_use = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    config_path = LaunchConfiguration("config_path")
    config_file = LaunchConfiguration("config_file")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        "rviz", default_value="true", description="Enable RViz visualization"
    )
    declare_rviz_config_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(default_config_path, "fast_lio_sam.rviz"),
        description="RViz config file path",
    )

    declare_config_path_cmd = DeclareLaunchArgument(
        "config_path",
        default_value=default_config_path,
        description="Yaml config file path",
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(default_config_path, "config.yaml"),
        description="Config file",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            get_package_share_directory("fast_lio_sam") + "/config/fast_lio_sam.rviz",
        ],
        condition=IfCondition(rviz_use),
        name="rviz_fast_lio_sam",
    )
    
    fast_lio_sam_node = Node(
        package="fast_lio_sam",
        executable="fast_lio_sam_node",
        output="screen",
        name="fast_lio_sam_node",
        parameters=[PathJoinSubstitution([config_path, config_file]), {'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(rviz_node)
    ld.add_action(fast_lio_sam_node)

    return ld
