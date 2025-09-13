from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # include lidar sim
    lidar_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fsd_vehicle_sim"),
                "launch",
                "lidar.launch.py",
            )
        )
    )

    # include perception tracking
    perception_tracking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fsd_perception_tracking"),
                "launch",
                "perception.launch.py",
            )
        )
    )

    # include vehicle sim
    vehicle_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fsd_vehicle_sim"),
                "launch",
                "sim.launch.py",
            )
        )
    )

    # include accel track
    accel_track_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fsd_accel_track"),
                "launch",
                "accel_track.launch.py",
            )
        )
    )

    # include localization
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fsd_localization"),
                "launch",
                "localization.launch.py",
            )
        )
    )

    # include vehicle description (TFs via robot_state_publisher)
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fsd_vehicle_description"),
                "launch",
                "description.launch.py",
            )
        )
    )

    # include trajectory
    trajectory_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fsd_trajectory_planning"),
                "launch",
                "trajectory.launch.py",
            )
        )
    )

    return LaunchDescription(
        [
            lidar_sim_launch,
            vehicle_sim_launch,
            description_launch,
            accel_track_launch,
            localization_launch,
            perception_tracking,
            trajectory_launch,
        ]
    )
