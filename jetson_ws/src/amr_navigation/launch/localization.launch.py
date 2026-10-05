from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    config = FindPackageShare("amr_navigation")
    xacro = PathJoinSubstitution([config, "urdf", "guide_amr.urdf.xacro"])
    ekf = PathJoinSubstitution([config, "config", "ekf.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"use_sim_time": use_sim_time,
                          "robot_description": Command(["xacro ", xacro])}]),
        Node(package="amr_navigation", executable="wheel_odometry_node",
             parameters=[{"use_sim_time": use_sim_time, "publish_tf": False}], output="screen"),
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node",
             parameters=[ekf, {"use_sim_time": use_sim_time}], output="screen"),
    ])
