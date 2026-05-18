cmake_minimum_required(VERSION 3.0.2)
project(px4_waypoint_circle_test)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  mavros_msgs
  mavros
)

catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs mavros_msgs mavros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/px4_waypoint_circle_test.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

