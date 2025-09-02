from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cfg = LaunchConfiguration("config")
    default_cfg = PathJoinSubstitution(
        [FindPackageShare("fsd_localization"), "config", "ekf.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_cfg,
                description="Path to EKF local YAML",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",  # in manchen Distros: ekf_node statt ekf_localization_node
                name="ekf_local",
                output="screen",
                parameters=[cfg],
            ),
        ]
    )
