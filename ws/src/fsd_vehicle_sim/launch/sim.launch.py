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
    ])
