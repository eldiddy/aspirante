cmake_minimum_required(VERSION 3.0.2)
project(aspirante)

find_package(catkin REQUIRED
  catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
  std_msgs
  tf
  urdf
  xacro
  nav_msgs
  actionlib
  move_base_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package()

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/coverage_path_planning_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(rviz_path_visualizer src/rviz_path_visualizer.cpp)
target_link_libraries(rviz_path_visualizer ${catkin_LIBRARIES} ${OpenCV_LIBS})

