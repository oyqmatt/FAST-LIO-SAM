import os.path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory("fast_lio_sam")
    default_lio_sam_config_path = os.path.join(package_path, "config")

    rviz_use = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    lio_sam_config_path = LaunchConfiguration("lio_sam_config_path")
    lio_sam_config_file = LaunchConfiguration("lio_sam_config_file")

    lidar = LaunchConfiguration("lidar")

    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Enable RViz visualization"
    )
    declare_rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(default_lio_sam_config_path, "sam_rviz.rviz"),
        description="RViz config file path",
    )

    declare_lidar_arg = DeclareLaunchArgument(
        "lidar",
        default_value="livox",
        description="Lidar type (ouster, velodyne, livox)",
    )
    declare_lio_sam_config_path_arg = DeclareLaunchArgument(
        "lio_sam_config_path",
        default_value=default_lio_sam_config_path,
        description="Yaml config file path",
    )
    declare_lio_sam_config_file_arg = DeclareLaunchArgument(
        "lio_sam_config_file",
        default_value=os.path.join(default_lio_sam_config_path, "config.yaml"),
        description="Config file",
    )

    rviz_lio_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            get_package_share_directory("fast_lio_sam") + "/config/lio_rviz.rviz",
        ],
        condition=IfCondition(rviz_use),
        name="rviz_lio",
    )

    rviz_sam_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            get_package_share_directory("fast_lio_sam") + "/config/sam_rviz.rviz",
        ],
        condition=IfCondition(rviz_use),
        name="rviz_sam",
    )
    
    fast_lio_sam_node = Node(
        package="fast_lio_sam",
        executable="fast_lio_sam_node",
        output="screen",
        name="fast_lio_sam_node",
        parameters=[PathJoinSubstitution([lio_sam_config_path, lio_sam_config_file])],
    )

    # mapping_ouster_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [
    #             get_package_share_directory("fast_lio"),
    #             "/launch/mapping_ouster64.launch.py",
    #         ]
    #     ),
    #     condition=IfCondition(("arg", "lidar") == "ouster"),
    #     launch_arguments={"rviz": "false"}.items(),
    # )

    # mapping_velodyne_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [
    #             get_package_share_directory("fast_lio"),
    #             "/launch/mapping_velodyne.launch.py",
    #         ]
    #     ),
    #     condition=IfCondition(("arg", "lidar") == "velodyne"),
    #     launch_arguments={"rviz": "false"}.items(),
    # )

    launch_livox_fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("fast_lio"), "/launch/mapping.launch.py"]
        ),
        launch_arguments={"rviz": "false", "use_sim_time": use_sim_time}.items(),
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_rviz_arg)
    ld.add_action(declare_rviz_config_arg)
    ld.add_action(declare_lidar_arg)
    ld.add_action(declare_lio_sam_config_file_arg)
    ld.add_action(declare_lio_sam_config_path_arg)
    # ld.add_action(rviz_lio_node)
    # ld.add_action(rviz_sam_node)
    ld.add_action(fast_lio_sam_node)
    # ld.add_action(mapping_ouster_launch)
    # ld.add_action(mapping_velodyne_launch)
    ld.add_action(launch_livox_fast_lio)

    return ld
