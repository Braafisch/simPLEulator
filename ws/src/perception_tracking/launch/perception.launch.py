from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="perception_tracking",
                executable="perception_tracking_node",
                name="fsd_perception_tracking",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/detection/cones",
                        "output_topic": "/perception/tracked_cones",
                        "source_frame": "lidar_link",
                        "target_frame": "odom",
                        "rate_hz": 10.0,
                        "gating_distance": 0.15,
                        "max_misses": 5,
                        "min_count": 3,
                        "process_var": 2.0,
                        "meas_var": 0.05,
                        "min_conf_new": 0.3,
                        "conf_inc": 0.1,
                        "conf_dec": 0.1,
                    }
                ],
            ),
        ]
    )
