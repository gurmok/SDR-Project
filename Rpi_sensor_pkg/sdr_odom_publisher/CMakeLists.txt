cmake_minimum_required(VERSION 2.8.3)
project(sdr_odom_publisher)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pagv_odom_publisher
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
)

 add_executable(sdr_odom_publisher_node_success src/sdr_odom_publisher_node_success.cpp)

 target_link_libraries(sdr_odom_publisher_node_success
   ${catkin_LIBRARIES} ${PCL_LIBRARIES}
 )
