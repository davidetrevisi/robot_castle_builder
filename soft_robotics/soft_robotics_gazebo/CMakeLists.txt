cmake_minimum_required(VERSION 2.8.3)
project(soft_robotics_gazebo)

find_package(
  catkin REQUIRED COMPONENTS
)

catkin_package()

 catkin_install_python(PROGRAMS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
