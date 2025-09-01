from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Use the package name from package.xml
    pkg_share = get_package_share_directory("fsd_vehicle_description")
    urdf_path = os.path.join(pkg_share, "urdf", "vehicle.urdf.xacro")

    # Sensor pose arguments (defaults match Xacro)
    args = [
        DeclareLaunchArgument("lidar_x", default_value="0.9"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="0.35"),
        DeclareLaunchArgument("lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
        DeclareLaunchArgument("gps_x", default_value="0.0"),
        DeclareLaunchArgument("gps_y", default_value="0.0"),
        DeclareLaunchArgument("gps_z", default_value="0.8"),
        DeclareLaunchArgument("gps_roll", default_value="0.0"),
        DeclareLaunchArgument("gps_pitch", default_value="0.0"),
        DeclareLaunchArgument("gps_yaw", default_value="0.0"),
    ]

    robot_description = ParameterValue(
        Command(
            [
                "xacro",
                " ",
                urdf_path,
                " ",
                "lidar_x:=",
                LaunchConfiguration("lidar_x"),
                " ",
                "lidar_y:=",
                LaunchConfiguration("lidar_y"),
                " ",
                "lidar_z:=",
                LaunchConfiguration("lidar_z"),
                " ",
                "lidar_roll:=",
                LaunchConfiguration("lidar_roll"),
                " ",
                "lidar_pitch:=",
                LaunchConfiguration("lidar_pitch"),
                " ",
                "lidar_yaw:=",
                LaunchConfiguration("lidar_yaw"),
                " ",
                "gps_x:=",
                LaunchConfiguration("gps_x"),
                " ",
                "gps_y:=",
                LaunchConfiguration("gps_y"),
                " ",
                "gps_z:=",
                LaunchConfiguration("gps_z"),
                " ",
                "gps_roll:=",
                LaunchConfiguration("gps_roll"),
                " ",
                "gps_pitch:=",
                LaunchConfiguration("gps_pitch"),
                " ",
                "gps_yaw:=",
                LaunchConfiguration("gps_yaw"),
            ]
        ),
        value_type=str,
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    return LaunchDescription(args + [rsp])
