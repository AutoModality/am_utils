cmake_minimum_required(VERSION 3.5)
project(am_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #to view [-Wunused-parameter] uncomment the following line
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(brain_box_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(jsoncpp REQUIRED)

pkg_check_modules(jsoncpp REQUIRED jsoncpp)

set(dependencies
        camera_info_manager
        OpenCV
        Eigen3
        brain_box_msgs
        builtin_interfaces
        rclcpp
        rclpy
        sensor_msgs
        stereo_msgs
        std_msgs
        nav_msgs
        tf2
        tf2_geometry_msgs
        tf2_ros
        pcl_conversions
        visualization_msgs
        rosbag2_cpp
        brain_box_msgs
        geometry_msgs
        mavros_msgs
        jsoncpp
        PCL
      )

include_directories(
					include
					${jsoncpp_INCLUDE_DIRS}
					${PCL_INCLUDE_DIRS}
					/usr/include/eigen3
  	)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})	


add_library(am_utils src/am_utils/am_statistics.cpp
					 src/am_utils/am_bag_utils.cpp
					 src/am_utils/am_network_utils.cpp
					 src/am_utils/am_onlinestatistics.cpp
					 src/am_utils/am_operator_utils.cpp
					 src/am_utils/am_pcl_utils.cpp
					 src/am_utils/cansocket.cpp
					 src/am_utils/dust_filter.cpp
					 src/am_utils/fcu_mode_type.cpp
					 src/am_utils/flightplan_type.cpp
					 src/am_utils/message_util.cpp
					 src/am_utils/mission_cmd_type.cpp
					 src/am_utils/operator_msg_type.cpp
					 src/am_utils/am_ros2_utility.cpp
					 
					 src/vb_util_lib/am_geometry.cpp
					 src/vb_util_lib/am_heartbeat.cpp
					 src/vb_util_lib/am_histogram.cpp
					 src/vb_util_lib/am_param.cpp
					 src/vb_util_lib/am_util.cpp
					 src/vb_util_lib/bag_logger.cpp
					 src/vb_util_lib/circ_list.cpp
					 src/vb_util_lib/control_util.cpp
					 src/vb_util_lib/csv_logger.cpp
					 src/vb_util_lib/current_enu_kinematics.cpp
					 src/vb_util_lib/file_spec.cpp
					 src/vb_util_lib/ftp_request.cpp
					 src/vb_util_lib/ftp_response.cpp
					 src/vb_util_lib/GeometryPoseExtractor.cpp
					 src/vb_util_lib/gis_entity.cpp
					 src/vb_util_lib/gis_util.cpp
					 src/vb_util_lib/gps_location.cpp
					 src/vb_util_lib/i2c.cpp
					 src/vb_util_lib/imu_class.cpp
					 #src/vb_util_lib/jsoncpp.cpp
					 #src/vb_util_lib/json_util.cpp
					 src/vb_util_lib/key_value_stamped.cpp
					 src/vb_util_lib/kinematics.cpp
					 src/vb_util_lib/kinematics_list.cpp
					 src/vb_util_lib/location_source.cpp
					 src/vb_util_lib/message.cpp
					 src/vb_util_lib/msg_data.cpp
					 src/vb_util_lib/msg_head.cpp
					 src/vb_util_lib/object_state.cpp
					 src/vb_util_lib/param_update.cpp
					 src/vb_util_lib/rc_controls.cpp
					 src/vb_util_lib/rc_mapper.cpp
					 src/vb_util_lib/rotate.cpp
				     src/vb_util_lib/row_marker.cpp
				     src/vb_util_lib/task_trigger.cpp
				     src/vb_util_lib/time_synchronizer.cpp
				     src/vb_util_lib/time_util.cpp
					 src/vb_util_lib/topics.cpp
					 #src/vb_util_lib/trajectory_data.cpp
					 src/vb_util_lib/transformer.cpp
					 src/vb_util_lib/bag_logger.cpp
					 )
target_link_libraries(am_utils ${OPENCV_LIBRARIES} ${PCL_LIBRARIES})
ament_target_dependencies(am_utils ${dependencies})
ament_export_targets(am_utilsTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(am_utils ${OPENCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(bag_logger_test src/nodes/bag_logger_test.cpp)
target_link_libraries(bag_logger_test am_utils)
ament_target_dependencies(bag_logger_test ${dependencies})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
install(DIRECTORY include/
        DESTINATION include/
    )
install(
  TARGETS am_utils
  EXPORT am_utilsTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  bag_logger_test
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(am_utils)
ament_export_dependencies(${dependencies})
ament_package()
