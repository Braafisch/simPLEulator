from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="fsd_vehicle_sim",
                executable="lidar_cone_sim",
                name="os1_like_lidar",
                output="screen",
                parameters=[
                    {
                        # Frames & Quelle
                        "lidar_frame": "lidar_link",
                        "cones_topic": "/visualization_marker_array",
                        "rate_hz": 20.0,  # nur Marker aus 'map' verwenden
                        # Reichweite / Filter
                        "min_range": 0.2,
                        "max_range": 90.0,
                        "h_fov_deg": 170.0,  # 360° Horizontal-FOV
                        "min_z": -1.0,
                        "max_z": 2.0,
                        # Rauschen / Dropouts / Intensität
                        "pos_noise_xy_std": 0.01,
                        "pos_noise_z_std": 0.00,
                        "dropout_prob": 0.0,
                        # Cone-Geometrie (für Verdeckung / Raycast)
                        "cone_radius": 0.15,
                        "cone_height": 0.5,
                    }
                ],
            ),
        ]
    )
