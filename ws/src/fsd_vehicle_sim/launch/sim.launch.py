from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="fsd_vehicle_sim",
                executable="bicycle_sim",
                name="bicycle_sim",
                output="screen",
                parameters=[
                    {
                        # typische Defaults – gern anpassen
                        "wheelbase": 1.6,
                        "rate_hz": 50.0,
                        "speed_limit": 15.0,
                        "steer_limit": 0.610865,  # 35° in rad, falls dein Code 'steer_limit' erwartet
                        "gps_lat0_deg": 48.999,
                        "gps_lon0_deg": 8.999,
                        "gps_alt0_m": 300.0,
                        "noise_gps_xy_std": 0.20,
                        "noise_gps_alt_std": 0.50,
                        "frame_map": "track",
                        "frame_base": "base_link",
                    }
                ],
            ),
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
                        "rate_hz": "20.0",  # nur Marker aus 'map' verwenden
                        # Reichweite / Filter
                        "min_range": 0.2,
                        "max_range": 90.0,
                        "h_fov_deg": 170.0,  # 360° Horizontal-FOV
                        "min_z": -1.0,
                        "max_z": 2.0,
                        # Rauschen / Dropouts / Intensität
                        "pos_noise_xy_std": 0.03,
                        "pos_noise_z_std": 0.02,
                        "dropout_prob": 0.0,
                        # Cone-Geometrie (für Verdeckung / Raycast)
                        "cone_radius": 0.15,
                        "cone_height": 0.5,
                    }
                ],
            ),
        ]
    )
