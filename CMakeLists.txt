cmake_minimum_required(VERSION 2.8.3)
project(am_utils)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  nav_msgs
  genmsg
  std_msgs
  sensor_msgs)

add_message_files( 
	DIRECTORY msg
	FILES 
	Latency_Image.msg
	Latency_LaserScan.msg
	Latency_Odometry.msg
	Latency_PointCloud.msg
	Latency_PointCloud2.msg
	Latency_PoseStamped.msg
	Latency_PoseWithCovarianceStamped.msg
	Latency_TwistStamped.msg
	Latency_TwistWithCovarianceStamped.msg
	Latency.msg
        Latency_String.msg
	LatencyHeader.msg
	LatencyTest.msg
        DelayStats.msg
	)
	
generate_messages( DEPENDENCIES std_msgs nav_msgs geometry_msgs sensor_msgs)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES am_utils
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    nav_msgs
    std_msgs
    sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(am_utils
  src/latency_wrapper.cpp src/am_statistics.cpp src/topics.cpp src/cansocket.cpp src/am_onlinestatistics.cpp 
 )
target_link_libraries(am_utils ${catkin_LIBRARIES})
add_dependencies(am_utils am_utils_generate_messages_cpp)
