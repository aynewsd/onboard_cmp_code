cmake_minimum_required(VERSION 3.0.2)
project(odom_to_out)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf2
)

catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs tf2
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_to_out_node src/odom_to_out_node.cpp)
target_link_libraries(odom_to_out_node ${catkin_LIBRARIES})

install(TARGETS odom_to_out_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

