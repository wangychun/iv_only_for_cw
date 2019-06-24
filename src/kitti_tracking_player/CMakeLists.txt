cmake_minimum_required(VERSION 2.8.3)
project(kitti_tracking_player)

SET (CMAKE_BUILD_TYPE Release)
SET (CMAKE_CXX_FLAGS "-O3 -std=c++11 -Wall")
find_package( OpenCV REQUIRED )
find_package(catkin REQUIRED COMPONENTS
                    roscpp
                    tf
                    tf2
                    std_msgs                    
                    geometry_msgs
                    cv_bridge
                    image_transport
                    dynamic_reconfigure
                    darknet_ros_msgs
)

find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system program_options)


generate_dynamic_reconfigure_options(cfg/kitti_player.cfg)
catkin_package(CATKIN_DEPENDS dynamic_reconfigure)

include_directories(
		SYSTEM  src include 
		${catkin_INCLUDE_DIRS}
  		${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(kitti_tracking_player src/kitti_tracking_player.cpp src/kitti_track_label.cpp)
target_link_libraries(kitti_tracking_player ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############
install(DIRECTORY launch DESTINATION share/kitti_tracking_player/)
install(DIRECTORY cfg DESTINATION share/kitti_tracking_player/)
 
install(TARGETS  kitti_tracking_player
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS   kitti_tracking_player
  EXPORT kitti_tracking_playerExport
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
