cmake_minimum_required(VERSION 2.8.3)
project(ir_normalization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  geometry_msgs
  image_geometry
  tf2_ros
  tf2_geometry_msgs
  tf
  camera_info_manager
)

find_package(OpenCV COMPONENTS opencv_highgui opencv_features2d opencv_xfeatures2d opencv_calib3d)
if(NOT OPENCV_FOUND)
	find_package(OpenCV REQUIRED COMPONENTS opencv_highgui opencv_features2d opencv_calib3d)
	add_definitions(-DNO_CV_XFEATURES2D)
endif()

#generate_dynamic_reconfigure_options(cfg/ir_normalization_params.cfg)

#list(APPEND OpenCV_LIBRARIES opencv_nonfree)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

catkin_package(
  LIBRARIES ir_normalization
  CATKIN_DEPENDS roscpp image_transport cv_bridge sensor_msgs  std_msgs geometry_msgs  
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(ir_normalization src/ir_normalization.cpp)
target_link_libraries(ir_normalization ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(ir_normalization ${PROJECT_NAME}_gencfg)

# install
install (TARGETS ir_normalization
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
