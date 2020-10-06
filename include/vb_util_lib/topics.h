#ifndef VB_UTIL_LIB_AM_TOPICS_H
#define VB_UTIL_LIB_AM_TOPICS_H

#include <string>

namespace am
{

class am_topics
{
public:
	static constexpr char DJI_SDK_IMU[] = "/dji_sdk/imu";
	static constexpr char DJI_SDK_IMU_RPY[] = "/dji_sdk/imu_rpy";
	static constexpr char DJI_SDK_RTK_POSITION[] = "/dji_sdk/rtk_position";
	static constexpr char DJI_SDK_RTK_VELOCITY[] = "/dji_sdk/rtk_velocity";
	static constexpr char DJI_SDK_RTK_YAW[] = "/dji_sdk/rtk_yaw";
	static constexpr char DJI_SDK_RC[] = "/dji_sdk/rc";
	static constexpr char DJI_SDK_GIMBAL_ANG_CMD[] = "/dji_sdk/gimbal_angle_cmd";
	static constexpr char DJI_SDK_GIMBAL_SPD_CMD[] = "/dji_sdk/gimbal_speed_cmd";
	static constexpr char DJI_SDK_GIMBAL_ANG[] = "/dji_sdk/gimbal_angle";
	static constexpr char DJI_SDK_GLOBAL_POSITION[] = "/dji_sdk/global_position";
	static constexpr char DJI_SDK_GPS_POSITION[] = "/dji_sdk/gps_position";
	static constexpr char DJI_SDK_FLIGHT_STATUS[] = "/dji_sdk/flight_status";
	static constexpr char DJI_SDK_BATTERY_STATE[] = "/dji_sdk/battery_state";
	static constexpr char DJI_SDK_CONTROL_STATUS[] = "/dji_sdk/control_status";
	static constexpr char DJI_SDK_FROM_MOBILE_DATA[] = "/dji_sdk/from_mobile_data";
	static constexpr char DJI_SDK_ACCELERATION_GROUND_FUSED[] = "/dji_sdk/acceleration_ground_fused";
	static constexpr char DJI_SDK_VO_POSIITON[] = "/dji_sdk/vo_position";
	static constexpr char DJI_SDK_VELOCITY[] = "/dji_sdk/velocity";
	static constexpr char DJI_SDK_ATTITUDE[] = "/dji_sdk/attitude";
	static constexpr char DJI_SDK_HEIGHT_ABOVE_TAKEOFF[] = "/dji_sdk/height_above_takeoff";
	static constexpr char DJI_SDK_ANGULAR_VELOCITY_FUSED[] = "/dji_sdk/angular_velocity_fused";
	static constexpr char DJI_SDK_RELATIVE_POSITION[] = "/dji_sdk/relative_position";
	static constexpr char DJI_SDK_ODOMETRY[] = "/dji_sdk/odometry";
	static constexpr char DJI_SDK_SEND_DATA_TO_MOBILE[] = "/dji_sdk/send_data_to_mobile";
	static constexpr char DJI_SDK_CORRECTED_ATTITUDE[] = "/dji_sdk/corrected_attitude";

	static constexpr char FEATURE_ODOMETRY[] = "/feature/odometry";
	static constexpr char FEATURE_SEARCH_IDS[] = "/feature/search_ids";

	static constexpr char SENSOR_LIDAR_SCAN[] = "/sensor/lidar/scan";
	static constexpr char SENSOR_LIDAR_SCAN_FILTERED[] = "/sensor/lidar/scan_filtered";
	static constexpr char SENSOR_LIDAR_POINTCLOUD[] = "/sensor/lidar/pointcloud";
	static constexpr char SENSOR_LIDAR_POINTCLOUD2[] = "/sensor/lidar/pointcloud2";

	static constexpr char SENSOR_OUSTER_POINTCLOUD2[] = "/sensor/ouster/pointcloud2";

	static constexpr char SENSOR_SS1_DEPTH_IMAGE_RECT[] = "/sensor/ss1/depth/image_rect";
	static constexpr char SENSOR_SS1_DEPTH_CAMERA_INFO[] = "/sensor/ss1/depth/camera_info";
	static constexpr char SENSOR_SS1_POINTCLOUD[] = "/sensor/ss1/pointcloud";
	static constexpr char SENSOR_SS1_POINTCLOUD2[] = "/sensor/ss1/pointcloud2";
	static constexpr char SENSOR_SS1_PCL[] = "/sensor/ss1/pcl";
	static constexpr char SENSOR_SS2_DEPTH_IMAGE_RECT[] = "/sensor/ss2/depth/image_rect";
	static constexpr char SENSOR_SS2_DEPTH_CAMERA_INFO[] = "/sensor/ss2/depth/camera_info";
	static constexpr char SENSOR_SS2_POINTCLOUD[] = "/sensor/ss2/pointcloud";
	static constexpr char SENSOR_SS2_POINTCLOUD2[] = "/sensor/ss2/pointcloud2";
	static constexpr char SENSOR_SS2_PCL[] = "/sensor/ss2/pcl";

	static constexpr char SENSOR_SC1_DEPTH_IMAGE_RECT[] = "/sensor/sc1/depth/image_rect";
	static constexpr char SENSOR_SC1_DEPTH_CAMERA_INFO[] = "/sensor/sc1/depth/camera_info";
	static constexpr char SENSOR_SC1_IMU[] = "/sensor/sc1/imu";
	static constexpr char SENSOR_SC1_IR_IMAGE[] = "/sensor/sc1/ir/image";
	static constexpr char SENSOR_SC1_IR_CAMERA_INFO[] = "/sensor/sc1/ir/camera_info";
	static constexpr char SENSOR_SC1_POINTCLOUD[] = "/sensor/sc1/pointcloud";
	static constexpr char SENSOR_SC1_VISIBLE_IMAGE[] = "/sensor/sc1/visible/image";
	static constexpr char SENSOR_SC1_VISIBLE_CAMERA_INFO[] = "/sensor/sc1/visible/camera_info";

	static constexpr char SENSOR_SC2_DEPTH_IMAGE_RECT[] = "/sensor/sc2/depth/image_rect";
	static constexpr char SENSOR_SC2_DEPTH_CAMERA_INFO[] = "/sensor/sc2/depth/camera_info";
	static constexpr char SENSOR_SC2_IMU[] = "/sensor/sc2/imu";
	static constexpr char SENSOR_SC2_IR_IMAGE[] = "/sensor/sc2/ir/image";
	static constexpr char SENSOR_SC2_IR_CAMERA_INFO[] = "/sensor/sc2/ir/camera_info";
	static constexpr char SENSOR_SC2_POINTCLOUD[] = "/sensor/sc2/pointcloud";
	static constexpr char SENSOR_SC2_VISIBLE_IMAGE[] = "/sensor/sc2/visible/image";
	static constexpr char SENSOR_SC2_VISIBLE_CAMERA_INFO[] = "/sensor/sc2/visible/camera_info";

	static constexpr char SENSOR_SC3_DEPTH_IMAGE_RECT[] = "/sensor/sc3/depth/image_rect";
	static constexpr char SENSOR_SC3_DEPTH_CAMERA_INFO[] = "/sensor/sc3/depth/camera_info";
	static constexpr char SENSOR_SC3_IMU[] = "/sensor/sc3/imu";
	static constexpr char SENSOR_SC3_IR_IMAGE[] = "/sensor/sc3/ir/image";
	static constexpr char SENSOR_SC3_IR_CAMERA_INFO[] = "/sensor/sc3/ir/camera_info";
	static constexpr char SENSOR_SC3_POINTCLOUD[] = "/sensor/sc3/pointcloud";
	static constexpr char SENSOR_SC3_VISIBLE_IMAGE[] = "/sensor/sc3/visible/image";
	static constexpr char SENSOR_SC3_VISIBLE_CAMERA_INFO[] = "/sensor/sc3/visible/camera_info";

	static constexpr char SENSOR_COLLISION[] = "/sensor/collision";
	static constexpr char SENSOR_COLLISION_VIEWS[] = "/sensor/collision_views";

	static constexpr char SENSOR_DISTANCE_AGL[] = "/sensor/distance/agl";
	static constexpr char SENSOR_DISTANCE_AGL_LW[] = "/sensor/distance/agl_lw";
	static constexpr char SENSOR_DISTANCE_DISTANCES_LW20[] = "/sensor/distance/distances_lw20";

    static constexpr char SENSOR_PROXIMITY[] = "/sensor/proximity";

        //dji vision topics generated by dji_vision nodes(raw and rectified for both ImageTransport and SharedMemory)
	//image_transport
	static constexpr char SENSOR_CAMERA_240P_RAW_FRONT_LEFT[] = "/sensor/camera/front_left/image_raw";
	static constexpr char SENSOR_CAMERA_240P_RAW_FRONT_RIGHT[] = "/sensor/camera/front_right/image_raw";
	static constexpr char SENSOR_CAMERA_240P_RAW_DOWN_FRONT[] = "/sensor/camera/down_front/image_raw";
	static constexpr char SENSOR_CAMERA_240P_RAW_DOWN_BACK[] = "/sensor/camera/down_back/image_raw";

	static constexpr char SENSOR_CAMERA_240P_RECT_FRONT_LEFT[] = "/sensor/camera/front_left/image_rect";
	static constexpr char SENSOR_CAMERA_240P_RECT_FRONT_RIGHT[] = "/sensor/camera/front_right/image_rect";
	static constexpr char SENSOR_CAMERA_240P_RECT_DOWN_FRONT[] = "/sensor/camera/down_front/image_rect";
	static constexpr char SENSOR_CAMERA_240P_RECT_DOWN_BACK[] = "/sensor/camera/down_back/image_rect";

	static constexpr char SENSOR_CAMERA_FPV_MAIN[] = "/sensor/camera/fpv_main/image_raw";


	//shared_memory topics
	static constexpr char SENSOR_CAMERA_SHM_240P_RAW_FRONT_LEFT[] = "/sensor/camera/shm/front_left/image_raw";
	static constexpr char SENSOR_CAMERA_SHM_240P_RAW_FRONT_RIGHT[] = "/sensor/camera/shm/front_right/image_raw";
	static constexpr char SENSOR_CAMERA_SHM_240P_RAW_DOWN_FRONT[] = "/sensor/camera/shm/down_front/image_raw";
	static constexpr char SENSOR_CAMERA_SHM_240P_RAW_DOWN_BACK[] = "/sensor/camera/shm/down_back/image_raw";

	static constexpr char SENSOR_CAMERA_SHM_240P_RECT_FRONT_LEFT[] = "/sensor/camera/shm/front_left/image_rect";
	static constexpr char SENSOR_CAMERA_SHM_240P_RECT_FRONT_RIGHT[] = "/sensor/camera/shm/front_right/image_rect";
	static constexpr char SENSOR_CAMERA_SHM_240P_RECT_DOWN_FRONT[] = "/sensor/camera/shm/down_front/image_rect";
	static constexpr char SENSOR_CAMERA_SHM_240P_RECT_DOWN_BACK[] = "/sensor/camera/shm/down_back/image_rect";

	static constexpr char SENSOR_CAMERA_SHM_FPV_MAIN[] = "/sensor/camera/shm/fpv_main/image_raw";

	static constexpr char SENSOR_CAMERA_IMAGE_CONTROLLER_IMAGE_RAW[] = "/sensor/camera/image_controller/image_raw";
	static constexpr char SENSOR_IMAGE_CONTROLLER_CAMERA_SELECT[] = "/image_controller/camera_select";

	//dji sdk camera services
	static constexpr char DJI_CAMERA_240P_SERVICE[] = "/dji_sdk/stereo_240p_subscription";
	static constexpr char DJI_CAMERA_VGA_SERVICE[] = "/dji_sdk/stereo_vga_subscription";
	static constexpr char DJI_CAMERA_FPV_MAIN_SERVICE[] = "/dji_sdk/setup_camera_stream";
	static constexpr char DJI_CAMERA_DEPTH_SERVICE[] = "/dji_sdk/stereo_depth_subscription";

	//dji sdk raw camera streams
	static constexpr char DJI_CAMERA_FPV_STREAM[] = "/dji_sdk/fpv_camera_images";
	static constexpr char DJI_CAMERA_MAIN_STREAM[] = "/dji_sdk/main_camera_images";
	static constexpr char DJI_CAMERA_240P_FRONT_LEFT_STREAM[] = "/dji_sdk/stereo_240p_front_left_images";
	static constexpr char DJI_CAMERA_240P_FRONT_RIGHT_STREAM[] = "/dji_sdk/stereo_240p_front_right_images";
	static constexpr char DJI_CAMERA_240P_DOWN_FRONT_STREAM[] = "/dji_sdk/stereo_240p_down_front_images";
	static constexpr char DJI_CAMERA_240P_DOWN_BACK_STREAM[] = "/dji_sdk/stereo_240p_down_back_images";
	static constexpr char DJI_CAMERA_VGA_FRONT_LEFT_STREAM[] = "/dji_sdk/stereo_vga_front_left_images";
	static constexpr char DJI_CAMERA_VGA_FRONT_RIGHT_STREAM[] = "/dji_sdk/stereo_vga_front_right_images";
	static constexpr char DJI_CAMERA_FRONT_DEPTH[] = "/dji_sdk/stereo_240p_front_depth_images";

	static constexpr char DJI_CAMERA_SHM_FPV_STREAM[] = "/dji_sdk/shm/fpv_camera_images";
	static constexpr char DJI_CAMERA_SHM_MAIN_STREAM[] = "/dji_sdk/shm/main_camera_images";
	static constexpr char DJI_CAMERA_SHM_240P_FRONT_LEFT_STREAM[] = "/dji_sdk/shm/stereo_240p_front_left_images";
	static constexpr char DJI_CAMERA_SHM_240P_FRONT_RIGHT_STREAM[] = "/dji_sdk/shm/stereo_240p_front_right_images";
	static constexpr char DJI_CAMERA_SHM_240P_DOWN_FRONT_STREAM[] = "/dji_sdk/shm/stereo_240p_down_front_images";
	static constexpr char DJI_CAMERA_SHM_240P_DOWN_BACK_STREAM[] = "/dji_sdk/shm/stereo_240p_down_back_images";
	static constexpr char DJI_CAMERA_SHM_VGA_FRONT_LEFT_STREAM[] = "/dji_sdk/shm/stereo_vga_front_left_images";
	static constexpr char DJI_CAMERA_SHM_VGA_FRONT_RIGHT_STREAM[] = "/dji_sdk/shm/stereo_vga_front_right_images";

	static constexpr char SENSOR_CAMERA_SELECT_SERVICE[] = "/sensor/camera/camera_select";

	static constexpr char SENSOR_LIDAR_GROUND_REMOVED_POINTCLOUD2[] = "/sensor/lidar/ground_removed/pointcloud2";
	static constexpr char SENSOR_LIDAR_POINTCLOUD2_WITH_NORMALS[] = "/sensor/lidar/pointcloud2/with_normals";

	static constexpr char STATUS_CEIL[] = "/status/ceil";
	static constexpr char STATUS_CORR[] = "/status/corr";
	static constexpr char STATUS_LIDAR[] = "/status/lidar";
	static constexpr char STATUS_SS1[] = "/status/ss1";
	static constexpr char STATUS_SS2[] = "/status/ss2";

	// visbox CTRL messages
	static constexpr char CTRL_VX_CMND[] = "/ctrl/vx/cmnd";
	static constexpr char CTRL_VX_FCU_STATUS[] = "/ctrl/vx/fcu_status";
	static constexpr char CTRL_VX_FLIGHT_CONTROL[] = "/ctrl/vx/flight_control";
	static constexpr char CTRL_VX_FLIGHTPLAN[] = "/ctrl/vx/flightplan";
	static constexpr char CTRL_VX_FLIGHTPAN_STATUS[] = "/ctrl/vx/flightplan/status";
	static constexpr char CTRL_VX_GLOBAL_POS_NAV_ACTION[] = "/ctrl/vx/global_pos_nav_action";
	static constexpr char CTRL_VX_GPS_LOCATION[] = "/ctrl/vx/gps_location";
	static constexpr char CTRL_VX_LOCAL_POS_NAV_ACTION[] = "/ctrl/vx/local_pos_nav_action";
	static constexpr char CTRL_VX_MISSION[] = "/ctrl/vx/mission";
	static constexpr char CTRL_VX_MISSION_MESSAGE[] = "/ctrl/vx/mission/message";
	static constexpr char CTRL_VX_MISSION_STATUS[] = "/ctrl/vx/mission_status";
	static constexpr char CTRL_VX_PID_PARAM[] = "/ctrl/vx/pid_param";
	static constexpr char CTRL_VX_PROXIMITY[] = "/ctrl/vx/proximity";
	static constexpr char CTRL_VX_RTL_ACTION[] = "/ctrl/vx/rtl_action";
	static constexpr char CTRL_VX_TARGET_CURRENT_ENU[] = "/ctrl/vx/target/currentENU";
	static constexpr char CTRL_VX_VEHICLE_CURRENTENU[] = "/ctrl/vx/vehicle/currentENU";
	static constexpr char CTRL_VX_VELOCITY_CMD[] = "/ctrl/vx/velocity_cmd";
	static constexpr char CTRL_VX_WAYPOINT_NAV_ACTION [] = "/ctrl/vx/waypoint_nav_action";

	static constexpr char CTRL_FEATURE_STATUS_LIST[] = "/ctrl/feature_status_list";
	static constexpr char CTRL_FEATURE_STATUS[] = "/ctrl/feature_status";
	static constexpr char CTRL_FEATURE_STATUS_UPDATE[] = "/ctrl/feature_status_update";
	static constexpr char CTRL_FEATURE_END_LIST[] =  "/ctrl/feature_end_list";

	static constexpr char CTRL_DEPTH_CMND[] = "/ctrl/depth/cmnd";
	static constexpr char CTRL_FTP[] = "/ctrl/ftp";
	static constexpr char CTRL_LOG_CONTROL[] = "/ctrl/log_control";
	static constexpr char CTRL_ROUTE[] = "/ctrl/route";
	static constexpr char CTRL_VIDEO_CMND[] = "/ctrl/video/cmnd";
	static constexpr char CTRL_VISION_CMND[] = "/ctrl/vision/cmnd";
	static constexpr char CTRL_VISION_MARKER_OFFSET[] = "/ctrl/vision/marker_offset";

	static constexpr char REMOTE_CTRL_VX_MISSION[] = "/remote/ctrl/vx/mission";

	static constexpr char LED_BLINK[] = "/blinkm_command";
	static constexpr char BATTERY_STATE[] = "/dji_sdk/battery_state";

	// Mavros specific topics
	static constexpr char MAVROS_RC_IN[] = "/mavros/rc/in";
	static constexpr char MAVROS_STATE[] = "/mavros/state";
	static constexpr char MAVROS_EXTENDED_STATE[] = "/mavros/extended_state";
	static constexpr char MAVROS_ARMING[] = "/mavros/cmd/arming";
	static constexpr char MAVROS_LANDING[] = "/mavros/cmd/land";
	static constexpr char MAVROS_TAKEOFF[] = "/mavros/cmd/takeoff";
	static constexpr char MAVROS_SET_MODE[] = "/mavros/set_mode";
	static constexpr char MAVROS_SETPOINT_LOCAL_RAW[] = "/mavros/setpoint_raw/local";
	static constexpr char MAVROS_SETPOINT_ATTITUDE_RAW[] = "/mavros/setpoint_raw/attitude";
	static constexpr char MAVROS_BATTERY[] = "/mavros/battery";
	static constexpr char MAVROS_GLOBAL_POSITION_GLOBAL[] = "/mavros/global_position/global";
	static constexpr char MAVROS_LOCAL_POSITION_POSE[] = "/mavros/local_position/pose";
	static constexpr char MAVROS_LOCAL_POSITION_VELOCITY_BODY[] = "/mavros/local_position/velocity_body";
	static constexpr char MAVROS_IMU_DATA[] = "/mavros/imu/data";

	static constexpr char ACSL_VO_VELOCITY[] = "/mavros/local_position/velocity_body";

	static std::string addBodySuffix(const char *topic);
	static std::string addBodySuffix(std::string topic);
	static std::string addBodyLevelSuffix(const char *topic);
	static std::string addBodyLevelSuffix(std::string topic);
	static std::string addLatencySuffix(const char *topic);
	static std::string addLatencySuffix(std::string topic);
	static std::string addDebugPrefix(const char *topic);
	static std::string addDebugPrefix(std::string topic);
};

}

#endif
