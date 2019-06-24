cmake_minimum_required(VERSION 2.8.3)
project(ivrc_cross_communication)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
   sensor_msgs
  roscpp
  roslib
  std_msgs
  iv_slam_ros_msgs
cv_bridge
)
find_package(ivcommon REQUIRED)
include("${IVCOMMON_CMAKE_DIR}/functions.cmake")

google_initialize_ivcommon_project()
include(FindPkgConfig)

include_directories(
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	)

catkin_package(
  CATKIN_DEPENDS geometry_msgs  roscpp roslib std_msgs iv_slam_ros_msgs cv_bridge
  DEPENDS EIGEN3  OpenCV 
  INCLUDE_DIRS src	
  LIBRARIES google_transform
)

add_library(google_transform src/rigid_transform.cc src/transform.cc)

add_executable(traversablearea_sub_main_demo src/traversablearea_sub_main_demo.cpp src/rigid_transform.cc src/transform.cc)
target_include_directories(traversablearea_sub_main_demo SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}" "${PCL_INCLUDE_DIRS}" "${catkin_INCLUDE_DIRS}")
target_link_libraries(traversablearea_sub_main_demo ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${EIGEN3_LIBRARIES} ivcommon)

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} -std=c++11")
set_target_properties(traversablearea_sub_main_demo google_transform PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

install(TARGETS  traversablearea_sub_main_demo  google_transform
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  TARGETS traversablearea_sub_main_demo google_transform
  EXPORT traversablearea_sub_main_demoExport
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
