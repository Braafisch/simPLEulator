from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="fsd_localization",
                executable="localization",
                name="localization",
                output="screen",
                parameters=[
                    {
                        "sensor_frame": "/vehicle/ground_speed",
                        "target_frame_tf": "base_link",
                        "source_frame_tf": "odom",
                    }
                ],
            ),
        ]
    )
