cmake_minimum_required(VERSION 2.8.3)
project(pff)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  geometry_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(src)

add_executable(velodyne src/velodyne.cpp)
add_executable(astra_pff src/astra_pff.cpp)
add_executable(kinect_pff src/kinect_pff.cpp)

target_link_libraries(velodyne ${catkin_LIBRARIES})
target_link_libraries(astra_pff ${catkin_LIBRARIES})
target_link_libraries(kinect_pff ${catkin_LIBRARIES})


