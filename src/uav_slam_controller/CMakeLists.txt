cmake_minimum_required(VERSION 3.0.2)
project(uav_slam_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
# 查找系统安装的jsoncpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs mavros_msgs tf2 tf2_ros tf2_geometry_msgs
  DEPENDS JSONCPP
)

# 头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

# 生成可执行文件
add_executable(slam_controller_node src/slam_controller_node.cpp)
# 链接库
target_link_libraries(slam_controller_node
  ${catkin_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  pthread
)

# 安装规则
install(TARGETS slam_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)