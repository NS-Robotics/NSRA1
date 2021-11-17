cmake_minimum_required(VERSION 3.0.2)
project(nsra_moveit_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  interactive_markers
  moveit_core
  moveit_commander
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  pluginlib
  geometric_shapes
  rosbag
  nsra_robot_vision
)


catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_commander
    nsra_robot_vision
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pick_place src/pick_place.cpp)
target_link_libraries(pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pick_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS
  src/nsra_moveit_interface
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
