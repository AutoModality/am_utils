cmake_minimum_required(VERSION 3.5)
project(am_utils)

set(ROS_BUILD_TYPE Debug)

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

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

set(dependencies
	brain_box_msgs
	builtin_interfaces
	camera_info_manager
	diagnostic_msgs
	diagnostic_updater
    Eigen3
    geometry_msgs
    jsoncpp
    mavros_msgs
    nav_msgs
    OpenCV
    PCL
    pcl_conversions
    rclcpp
    rclpy
    rosbag2
    rosbag2_cpp
    rosbag2_storage
    sensor_msgs
    std_msgs
    stereo_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

pkg_check_modules(jsoncpp REQUIRED jsoncpp)
	
include_directories(
	include
	${jsoncpp_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	/usr/include/eigen3
	)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})	

file(GLOB super_lib_cpp_files
    src/super_lib/*.cpp
)

file(GLOB am_utils_lib_cpp_files
    src/am_utils/*.cpp
)

file(GLOB vb_util_lib_cpp_files
    src/vb_util_lib/*.cpp
)

add_library(am_utils 
  ${super_lib_cpp_files} 
  ${am_utils_lib_cpp_files}
  ${vb_util_lib_cpp_files}
)

target_link_libraries(am_utils gtest ${OPENCV_LIBRARIES} ${PCL_LIBRARIES})
ament_target_dependencies(am_utils ${dependencies})
ament_export_targets(am_utilsTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(am_utils gtest ${OPENCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(bag_logger_test src/nodes/bag_logger_test.cpp)
target_link_libraries(bag_logger_test am_utils)
ament_target_dependencies(bag_logger_test ${dependencies})

add_executable(node_killer src/nodes/node_killer.cpp)
target_link_libraries(node_killer am_utils)
ament_target_dependencies(node_killer ${dependencies})

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
  node_killer
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
