################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(amr)


################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)





################################################################################
# Install
################################################################################
install(DIRECTORY rviz description
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
