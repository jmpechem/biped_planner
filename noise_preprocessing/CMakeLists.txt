cmake_minimum_required(VERSION 2.8.3)
project(noise_preprocessing)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nexus
  CATKIN_DEPENDS
#  DEPENDS system_lib
)

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
add_executable(noise_preprocessing src/noise_preprocessing.cpp)
target_link_libraries(noise_preprocessing
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )

