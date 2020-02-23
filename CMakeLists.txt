cmake_minimum_required(VERSION 2.8.3)
project(am_utils)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  genmsg
  std_msgs
  sensor_msgs
  rosconsole
  rostime
  brain_box_msgs
  )

generate_messages( DEPENDENCIES std_msgs nav_msgs geometry_msgs sensor_msgs)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include include/am_utils
  LIBRARIES am_utils
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    std_msgs
    sensor_msgs
    brain_box_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_library(am_utils
  src/am_bag_utils.cpp
  src/am_network_utils.cpp
  src/am_onlinestatistics.cpp 
  src/am_operator_utils.cpp
  src/am_statistics.cpp 
  src/cansocket.cpp 
  src/fcu_mode_type.cpp
  src/flightplan_type.cpp
  src/latency_wrapper.cpp 
  src/message_util.cpp
  src/mission_cmd_type.cpp
  src/operator_msg_type.cpp
  src/topics.cpp 
 )

target_link_libraries(am_utils ${catkin_LIBRARIES})

add_dependencies(am_utils brain_box_msgs_generate_messages_cpp)

install(TARGETS am_utils 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
