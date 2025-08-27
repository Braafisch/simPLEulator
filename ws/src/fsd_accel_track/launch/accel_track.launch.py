from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_world_to_map',
            arguments=['0','0','0','0','0','0','world','map']
        ),
        Node(
            package='fsd_accel_track',
            executable='accel_track_markers',
            name='accel_track_markers',
            output='screen',
            parameters=[{
                'frame_id': 'map',
                'length': 75.0,
                'spacing': 5.0,
                'lane_half_width': 1.5,
                'radius': 0.15,
                'height': 0.5,
                'rate_hz': 2.0,
            }]
        )
    ])
