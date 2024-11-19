cmake_minimum_required(VERSION 3.0.2)
project(lidar_tool)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

find_package(PCL 1.10 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES read_pcd
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(map_show src/map_show.cpp)
target_link_libraries(map_show ${catkin_LIBRARIES})

add_executable(cloud_filter src/cloud_filter.cpp)
target_link_libraries(cloud_filter 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})


add_executable(merge_lidar src/merge_lidar.cpp)
target_link_libraries(merge_lidar 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})
