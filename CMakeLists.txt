cmake_minimum_required(VERSION 2.8.3)
project(am_utils)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  brain_box_msgs
  control_toolbox 
  dji_sdk
  image_transport 
  genmsg
  geometry_msgs
  nav_msgs
  rosconsole
  roscpp 
  rosbag
  rostime
  sensor_msgs
  std_msgs
  tf2
  tf2_ros	
  )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include include/am_utils include/vb_util_lib
  LIBRARIES am_utils
  CATKIN_DEPENDS
  brain_box_msgs
  control_toolbox 
  dji_sdk
  image_transport 
  genmsg
  geometry_msgs
  nav_msgs
  rosconsole
  roscpp 
  rosbag
  rostime
  sensor_msgs
  std_msgs
  tf2
  tf2_ros	
)

# TODO: why is the explicit eigen3 include 
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  /usr/include/eigen3
)

add_library(am_utils
  src/am_utils/am_bag_utils.cpp
  src/am_utils/am_network_utils.cpp
  src/am_utils/am_onlinestatistics.cpp 
  src/am_utils/am_operator_utils.cpp
  src/am_utils/am_statistics.cpp 
  src/am_utils/cansocket.cpp 
  src/am_utils/fcu_mode_type.cpp
  src/am_utils/flightplan_type.cpp
  src/am_utils/latency_wrapper.cpp 
  src/am_utils/message_util.cpp
  src/am_utils/mission_cmd_type.cpp
  src/am_utils/operator_msg_type.cpp
  src/vb_util_lib/control_util.cpp
  src/vb_util_lib/object_state.cpp 
  src/vb_util_lib/kinematics.cpp 
  src/vb_util_lib/kinematics_list.cpp
  src/vb_util_lib/current_enu_kinematics.cpp 
  src/vb_util_lib/am_param.cpp 
  src/vb_util_lib/gis_entity.cpp 
  src/vb_util_lib/gis_util.cpp
  src/vb_util_lib/jsoncpp.cpp
  src/vb_util_lib/json_utils.cpp 
  src/vb_util_lib/gs_json_utils.cpp 
  src/vb_util_lib/am_util.cpp 
  src/vb_util_lib/key_value_stamped.cpp 
  src/vb_util_lib/param_update.cpp
  src/vb_util_lib/gps_util.cpp 
  src/vb_util_lib/location_source.cpp
  src/vb_util_lib/am_geometry.cpp 
  src/vb_util_lib/topics.cpp
  src/vb_util_lib/task_trigger.cpp 
  src/vb_util_lib/time_util.cpp 
  src/vb_util_lib/ftp_request.cpp 
  src/vb_util_lib/ftp_response.cpp 
  src/vb_util_lib/file_spec.cpp 
  src/vb_util_lib/message.cpp
  src/vb_util_lib/msg_head.cpp 
  src/vb_util_lib/msg_data.cpp 
  src/vb_util_lib/dji_challenge_util.cpp 
  src/vb_util_lib/am_histogram.cpp
  src/vb_util_lib/row_marker.cpp 
  src/vb_util_lib/rc_controls.cpp 
  src/vb_util_lib/i2c.cpp 
  src/vb_util_lib/circ_list.cpp 
  src/vb_util_lib/time_synchronizer.cpp
  src/vb_util_lib/GeometryPoseExtractor.cpp 
  src/vb_util_lib/trajectory_data.cpp 
  src/vb_util_lib/transformer.cpp 
  src/vb_util_lib/rotate.cpp 
  src/vb_util_lib/imu_class.cpp
  src/vb_util_lib/am_heartbeat.cpp
  src/vb_util_lib/vb_main.cpp
  src/vb_util_lib/bag_logger.cpp)

target_link_libraries(am_utils ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS am_utils
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/am_utils include/vb_util_lib
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

  
