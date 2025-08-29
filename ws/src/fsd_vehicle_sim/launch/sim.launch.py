from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fsd_vehicle_sim',
            executable='bicycle_sim',
            name='bicycle_sim',
            output='screen',
            parameters=[{
                # typische Defaults – gern anpassen
                'wheelbase': 1.6,
                'rate_hz': 50.0,
                'speed_limit': 15.0,
                'steer_limit': 0.610865,   # 35° in rad, falls dein Code 'steer_limit' erwartet
                'gps_lat0_deg': 48.999,
                'gps_lon0_deg': 8.999,
                'gps_alt0_m': 300.0,
                'noise_gps_xy_std': 0.20,
                'noise_gps_alt_std': 0.50,
                'frame_map': 'map',
                'frame_base': 'base_link',
            }],
        ),
         # --- TF: base_link -> lidar_link (Position des LiDAR an der Nasenspitze o.ä.)
        # Beispiel: 0.9 m vorwärts, 0.35 m hoch, keine Rotation
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_lidar',
            arguments=['0.9', '0.0', '0.35', '0', '0', '0', 'base_link', 'lidar_link'],
            output='screen'
        ),

        # --- TF: base_link -> gps (Antenne). Setz die Offsets passend (hier: über dem Schwerpunkt)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_gps',
            arguments=['0.0', '0.0', '0.8', '0', '0', '0', 'base_link', 'gps'],
            output='screen'
        ),
        Node(
            package='fsd_vehicle_sim',
            executable='lidar_cone_sim',   # <-- nicht "bicycle_sim"!
            name='os1_like_lidar',
            output='screen',
            parameters=[{
                # Frames & Quelle
                'lidar_frame': 'lidar_link',
                'cones_topic': '/visualization_marker_array',
                'accept_only_frame': 'map',   # nur Marker aus 'map' verwenden

                # OS1-ähnliches Scanmuster
                'rings': 32,
                'vert_fov_deg': 42.4,
                'spin_hz': 10.0,              # 10 oder 20 Hz
                'columns_per_rev': 1024,

                # Reichweite / Filter
                'min_range': 0.2,
                'max_range': 90.0,
                'h_fov_deg': 360.0,           # 360° Horizontal-FOV
                'min_z': -1.0,
                'max_z':  2.0,

                # Rauschen / Dropouts / Intensität
                'range_noise_std': 0.02,
                'dropout_prob': 0.0,
                'intensity_base': 80.0,
                'intensity_decay_per_m': 0.5,

                # Cone-Geometrie (für Verdeckung / Raycast)
                'cone_radius': 0.15,
                'cone_height': 0.5,

                # Dual Return optional
                'dual_return': False,
            }],
        )
    ])
