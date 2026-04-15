cmake_minimum_required(VERSION 3.0.2)
project(uav_gazebo_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  urdf
  xacro
  gazebo_ros
  gazebo_plugins
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs urdf xacro gazebo_ros gazebo_plugins
)

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY urdf worlds launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)