from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    share = FindPackageShare("amr_navigation")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, "launch", "localization.launch.py"])),
            launch_arguments={"use_sim_time": use_sim_time}.items()),
        Node(package="slam_toolbox", executable="async_slam_toolbox_node",
             name="slam_toolbox", parameters=[PathJoinSubstitution([share, "config", "slam.yaml"]),
                                                {"use_sim_time": use_sim_time}], output="screen"),
    ])
