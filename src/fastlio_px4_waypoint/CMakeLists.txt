cmake_minimum_required(VERSION 3.0.2)
project(fastlio_px4_waypoint)

## 编译选项
add_compile_options(-std=c++11)

## 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  mavros_msgs
  mavros
)

## catkin配置
catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs mavros_msgs mavros
)

## 包含头文件
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 编译可执行文件
add_executable(${PROJECT_NAME}_node src/fastlio_px4_waypoint.cpp)

## 链接库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

## 安装
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)