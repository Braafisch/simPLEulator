cmake_minimum_required(VERSION 3.8)
project(fsd_localization)

find_package(ament_cmake REQUIRED)

# Install launch & config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
