from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")
    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", default_value="automatic"),
            ExecuteProcess(
                cmd=["python3", "-m", "monitoring.server", "--host", "127.0.0.1"],
                output="screen",
            ),
            Node(
                package="amr_dummy",
                executable="dummy_sensor_node",
                parameters=[{"scenario": scenario, "publish_rate_hz": 20.0}],
                output="screen",
            ),
            Node(
                package="amr_dummy",
                executable="dummy_mcu_node",
                parameters=[{"update_rate_hz": 20.0}],
                output="screen",
            ),
            Node(
                package="amr_safety_node",
                executable="safety_controller_node",
                parameters=[{"drive_enable": True}],
                output="screen",
            ),
            Node(
                package="amr_monitoring_adapter",
                executable="monitoring_adapter_node",
                output="screen",
            ),
        ]
    )
