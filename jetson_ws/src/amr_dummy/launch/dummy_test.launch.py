from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")
    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", default_value="automatic"),
            Node(
                package="amr_dummy",
                executable="dummy_sensor_node",
                parameters=[{"scenario": scenario}],
                output="screen",
            ),
            Node(
                package="amr_dummy",
                executable="dummy_mcu_node",
                output="screen",
            ),
        ]
    )
