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

pkg_check_modules(jsoncpp REQUIRED jsoncpp)

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
					 
					 
					 src/vb_util_lib/am_geometry.cpp
					 src/vb_util_lib/topics.cpp
					 src/vb_util_lib/rotate.cpp
					 src/vb_util_lib/csv_logger.cpp
					 src/vb_util_lib/am_heartbeat.cpp
					 src/vb_util_lib/am_histogram.cpp
					 src/vb_util_lib/am_param.cpp
					 src/vb_util_lib/am_util.cpp
					 src/vb_util_lib/circ_list.cpp
					 #src/vb_util_lib/bag_logger.cpp
					 )
ament_target_dependencies(am_utils geometry_msgs brain_box_msgs PCL)




ament_export_include_directories(include)
ament_export_libraries(am_utils)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

ament_package()
