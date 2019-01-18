cmake_minimum_required(VERSION 2.8.3)
project(position_combine)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  tf
  roslib
  mrpt_msgs
)

find_package(Qt4 4.6 REQUIRED COMPONENTS QtCore QtGui QtNetwork QtSql)
include( ${QT_USE_FILE})
set(HDRS

include/position_combine.h

)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES slam_test
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
  DEPENDS system_lib
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
  ${QT_INCLUDE_DIRS}
  /usr/include/qt4/QtCore/
   /usr/include/qt4/
)

## Declare a C++ executable
add_executable(position_combine_node src/position_combine.cpp)
target_link_libraries(position_combine_node ${catkin_LIBRARIES} ${QT_LIBRARIES})
##add_dependencies(position_combine_node anbot_msgs_gencpp)
