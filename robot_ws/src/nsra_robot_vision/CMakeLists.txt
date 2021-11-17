cmake_minimum_required(VERSION 3.0.2)
project(nsra_robot_vision)

set(OpenCV_DIR /usr/share/opencv4/)

find_package(OpenCV 4.2 EXACT REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  roslib
  std_msgs
  message_generation
  nsra_odrive_interface
)

add_service_files(DIRECTORY srv FILES coords.srv)
add_message_files(DIRECTORY msg FILES stereo_camera_coords.msg)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS roslib roscpp std_msgs message_runtime nsra_odrive_interface
)

include_directories(
  include/
  config/
  /usr/local/lib
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(/usr/local/lib)

add_executable(calib_intrinsics src/camera_callibration/calib_intrinsics.cpp)

target_link_libraries(calib_intrinsics
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

#add_executable(calibrate src/deprecated/calib_intrinsic.cpp include/popt_pp.h)
#target_link_libraries(calibrate ${OpenCV_LIBS} "-lpopt")

#add_executable(read src/deprecated/read_images.cpp)
#target_link_libraries(read ${catkin_LIBRARIES} ${OpenCV_LIBS} "-lpopt" -lpthread)

add_executable(calib_stereo src/camera_callibration/calib_stereo.cpp)
target_link_libraries(calib_stereo ${catkin_LIBRARIES} ${OpenCV_LIBS} "-lpopt")

#add_executable(undistort_rectify src/calibration/undistort_rectify.cpp)
#target_link_libraries(undistort_rectify ${catkin_LIBRARIES} ${OpenCV_LIBS} "-lpopt")

add_executable(nsra_bottle_detection src/nsra_bottle_detection.cpp)
target_link_libraries(nsra_bottle_detection ${catkin_LIBRARIES} ${OpenCV_LIBS} "-lpopt" libndi.so)

install(PROGRAMS
  src/corner_detection/cbc_detection_left
  src/corner_detection/cbc_detection_right
  src/nsra_gripper
  src/bottle_detection_v1/nsra_bd_top_view
  src/bottle_detection_v1/nsra_bd_object_detection
  src/bottle_detection_v1/nsra_bd_front_view
  src/camera_callibration/calib_ingest
  src/tests/stereo_camera
  src/bottle_detection_v2/nsra_stereo_bottle_detection
  src/bottle_detection_v2/nsra_color_configuration
  src/tests/stereo_depth_map
  src/deprecated/NDI_source_viewer
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  calib_intrinsics
  #calibrate
  #read
  calib_stereo
  nsra_bottle_detection
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY lib/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/lib
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY include/   
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})