################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(open_manipulator_with_tb3_tools)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  open_manipulator_msgs
  open_manipulator_with_tb3_msgs
  ar_track_alvar_msgs
)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS rospy roscpp std_msgs geometry_msgs nav_msgs open_manipulator_msgs open_manipulator_with_tb3_msgs ar_track_alvar_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pick src/pick.cpp)
add_dependencies(pick ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pick ${catkin_LIBRARIES})

add_executable(pick2 src/pick2.cpp)
add_dependencies(pick2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pick2 ${catkin_LIBRARIES})

add_executable(place src/place.cpp)
add_dependencies(place ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(place ${catkin_LIBRARIES})

add_executable(place2 src/place2.cpp)
add_dependencies(place2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(place2 ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS
  nodes/controller_for_pick_and_place
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS pick pick2 place place2
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch maps rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
