# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/jsoncpp".split(';') if "${prefix}/include;/usr/include/jsoncpp" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;std_msgs;geometry_msgs;nav_msgs;sensor_msgs;mavros_msgs;tf2;tf2_ros;tf2_geometry_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ljsoncpp".split(';') if "-ljsoncpp" != "" else []
PROJECT_NAME = "uav_slam_controller"
PROJECT_SPACE_DIR = "/home/demo/uav_ws/install"
PROJECT_VERSION = "0.0.1"
