cmake_minimum_required(VERSION 3.8)
project(fsd_vehicle_description)

find_package(ament_cmake REQUIRED)

# Install launch and URDF/Xacro so ros2 launch and get_package_share_directory can find them
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.py")

install(DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME})

ament_package()
