from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    share = FindPackageShare("amr_navigation")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map", description="Absolute path to the saved map YAML"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, "launch", "localization.launch.py"])),
            launch_arguments={"use_sim_time": use_sim_time}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"])),
            launch_arguments={
                "map": map_file, "use_sim_time": use_sim_time, "autostart": "true",
                "params_file": PathJoinSubstitution([share, "config", "nav2.yaml"]),
            }.items()),
        Node(package="topic_tools", executable="relay", name="nav_cmd_relay",
             arguments=["/cmd_vel", "/cmd_vel_raw"], output="screen"),
    ])
