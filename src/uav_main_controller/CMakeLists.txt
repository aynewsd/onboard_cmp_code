cmake_minimum_required(VERSION 3.0.2)
project(uav_main_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
  servo_controller
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs mavros_msgs servo_controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mission_controller_node src/mission_controller_node.cpp)
target_link_libraries(mission_controller_node ${catkin_LIBRARIES})

install(TARGETS mission_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

