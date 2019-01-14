#ifndef AM_UTILS_AM_TOPICS_H
#define AM_UTILS_AM_TOPICS_H

namespace am
{

class am_topics
{
public:
	static constexpr char DJI_SDK_IMU[] = "/dji_sdk/imu";
	static constexpr char DJI_SDK_RTK_POSITION[] = "/dji_sdk/rtk_position";

	static constexpr char FEATURE_ODOMETRY[] = "/feature/odometry";
	static constexpr char FEATURE_SEARCH_IDS[] = "/feature/search_ids";

	static constexpr char SENSOR_LIDAR_SCAN[] = "/sensor/lidar/scan";
	static constexpr char SENSOR_LIDAR_SCAN_FILTERED[] = "/sensor/lidar/scan_filtered";
	static constexpr char SENSOR_LIDAR_POINTCLOUD[] = "/sensor/lidar/pointcloud";
	static constexpr char SENSOR_LIDAR_POINTCLOUD2[] = "/sensor/lidar/pointcloud2";

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

	static constexpr char SENSOR_DISTANCE_AGL[] = "/sensor/distance/agl";
	static constexpr char SENSOR_DISTANCE_DISTANCES_LW20[] = "/sensor/distance/distances_lw20";

	static constexpr char STATUS_CEIL[] = "/status/ceil";
	static constexpr char STATUS_CORR[] = "/status/corr";
	static constexpr char STATUS_LIDAR[] = "/status/lidar";
	static constexpr char STATUS_SS1[] = "/status/ss1";
	static constexpr char STATUS_SS2[] = "/status/ss2";

	static constexpr char LED_BLINK[] = "/blinkm_command";

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
