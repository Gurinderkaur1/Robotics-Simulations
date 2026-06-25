cmake_minimum_required(VERSION 3.0.2)
project(delivery_robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    geometry_msgs 
    sensor_msgs 
    nav_msgs 
    tf2_ros 
    tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/rc.py
  scripts/oa.py
  scripts/ki.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
