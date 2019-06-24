cmake_minimum_required(VERSION 2.8.3)
project(dynamic_object)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -O1 -std=gnu++0x")
set(CMAKE_BUILD_TYPE "Release")
set(PACKAGE_DEPENDENCIES
  eigen_conversions
  geometry_msgs
  message_runtime
  nav_msgs
  rosbag
  roscpp
  roslib
  sensor_msgs
  std_msgs
  visualization_msgs
  cv_bridge  
  image_transport
  tf
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(ivcommon REQUIRED)
find_package(iv_dynamicobject_msgs REQUIRED)
set(BUILD_SHARED_LIBS OFF)
include("${IVCOMMON_CMAKE_DIR}/functions.cmake")
google_initialize_ivcommon_project()

include(FindPkgConfig)
find_package(LuaGoogle REQUIRED)
find_package(PCL REQUIRED)
#set(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(Helper REQUIRED)

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS src
)
include_directories(SYSTEM  src include ${catkin_INCLUDE_DIRS} ${ivcommon_INCLUDE_DIRS} ${LUA_INCLUDE_DIR}
${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${iv_dynamicobject_msgs_INCLUDE_DIRS}
${Helper_INCLUDE_DIRS} )

###########
## Build ##
###########
add_library(dense_point_cloud_generation SHARED src/dense_point_cloud_generation.cpp)
target_link_libraries(dense_point_cloud_generation PUBLIC ${catkin_LIBRARIES}  ${ivcommon_LIBRARIES}  ${PCL_LIBRARIES}  ${Boost_LIBRARIES}  ${OpenCV_LIBS} glog gflags )

add_library(detector SHARED src/detect/ground_removal.cpp 
src/detect/gaus_blur.cpp 
src/detect/component_clustering.cpp
src/detect/box_fitting.cpp
src/detect/ogm_detector.cpp 
)
target_link_libraries(detector PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_library(motion_update SHARED src/motion/TargetTracking.cpp  
src/motion/TrajectoryDetection.cpp 
src/motion/Motion_global.cpp)
target_link_libraries(motion_update PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_library(obj_matcher SHARED src/match/TargetMatching.cpp
src/match/object_matching.cpp)
target_link_libraries(obj_matcher PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_library(sensors_fusion_utils SHARED src/utils/bbox_2d.cpp 
src/utils/cloud_process_utils.cpp
src/utils/lidar_camera_projection_utils.cpp
src/utils/messages_sync.cpp
src/utils/ogm_mapping_utils.cpp
src/StructMovingTargetDefine.cpp 
src/tracker/ukf.cpp
src/visualization/ShowResult.cpp
src/visualization/visualizer_utils.cpp
src/visualization/visualization.cpp
)
target_link_libraries(sensors_fusion_utils PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES} 
${OpenCV_LIBS} ${Boost_LIBRARIES} ${Helper_LIBRARIES})

add_library(tracker_utils SHARED src/tracker/ukf.cpp
src/tracker/multi_object_tracking.cpp
src/tracker/MovingTargetTrack.cpp
)
target_link_libraries(tracker_utils PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} detector obj_matcher motion_update sensors_fusion_utils)

add_library(sensors_fusion SHARED
src/sensors_fusion/fusion_detection.cpp)
target_link_libraries(sensors_fusion PUBLIC ${catkin_LIBRARIES} ${Boost_LIBRARIES} tracker_utils sensors_fusion_utils)

add_executable(dynamic_object_kitti_node src/dynamic_object_kitti_node.cpp)
target_link_libraries(dynamic_object_kitti_node PUBLIC  ${catkin_LIBRARIES}  ${ivcommon_LIBRARIES}  ${PCL_LIBRARIES}  ${Boost_LIBRARIES}  ${OpenCV_LIBS} 
detector motion_update obj_matcher tracker_utils sensors_fusion)

add_executable(fusion_detection_node src/fusion_detection_node.cpp)
target_link_libraries(fusion_detection_node PUBLIC  ${catkin_LIBRARIES}  ${ivcommon_LIBRARIES}  ${PCL_LIBRARIES}  ${Boost_LIBRARIES}  ${OpenCV_LIBS} 
detector motion_update obj_matcher tracker_utils sensors_fusion)

add_executable(dynamic_object_tracking_node src/dynamic_object_tracking_node.cpp)
target_link_libraries(dynamic_object_tracking_node PUBLIC  ${catkin_LIBRARIES} ${ivcommon_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
detector motion_update obj_matcher tracker_utils sensors_fusion)

add_executable(messages_sync_node src/messages_sync_node.cpp)
target_link_libraries(messages_sync_node PUBLIC  ${catkin_LIBRARIES} ${PCL_LIBRARIES} sensors_fusion_utils)

add_executable(ros_test_node src/ros_test_node.cpp)
target_link_libraries(ros_test_node PUBLIC  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#############
## Install ##
#############
install(DIRECTORY launch DESTINATION share/dynamic_object/)
install(DIRECTORY config DESTINATION share/dynamic_object/)

install(TARGETS  dense_point_cloud_generation detector motion_update obj_matcher  tracker_utils sensors_fusion sensors_fusion_utils
				dynamic_object_kitti_node dynamic_object_tracking_node messages_sync_node ros_test_node
				 fusion_detection_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS  dense_point_cloud_generation detector motion_update obj_matcher  tracker_utils sensors_fusion  sensors_fusion_utils
          dynamic_object_kitti_node dynamic_object_tracking_node messages_sync_node ros_test_node
           fusion_detection_node
  EXPORT dynamic_objectExport
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


# Install source headers.
file(GLOB_RECURSE hdrs "*.h" "*.hpp")

foreach(HDR ${hdrs})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)

  install(
    FILES
      ${HDR}
    DESTINATION
      include/dynamic_object/${INSTALL_DIR}
  )
endforeach()


set(DynamicObject_CMAKE_DIR share/dynamic_object/cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  dynamic_object-config.cmake.in
  ${PROJECT_BINARY_DIR}/cmake/dynamic_object/dynamic_object-config.cmake
  PATH_VARS DynamicObject_CMAKE_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/dynamic_object
)

install(
  EXPORT dynamic_objectExport
  DESTINATION share/dynamic_object/cmake/
  FILE DynamicObjectTargets.cmake
)

install(
  FILES ${PROJECT_BINARY_DIR}/cmake/dynamic_object/dynamic_object-config.cmake
  DESTINATION share/dynamic_object/
)
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
