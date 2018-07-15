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
	static constexpr char SENSOR_LIDAR_POINTCLOUD[] = "/sensor/lidar/pointcloud";
	static constexpr char SENSOR_SS1_DEPTH_IMAGE_RECT[] = "/sensor/ss1/depth/image_rect";
	static constexpr char SENSOR_SS1_DEPTH_CAMERA_INFO[] = "/sensor/ss1/depth/camera_info";
	static constexpr char SENSOR_SS1_POINTCLOUD[] = "/sensor/ss1/pointcloud";
	static constexpr char SENSOR_SS2_DEPTH_IMAGE_RECT[] = "/sensor/ss2/depth/image_rect";
	static constexpr char SENSOR_SS2_DEPTH_CAMERA_INFO[] = "/sensor/ss2/depth/camera_info";
	static constexpr char SENSOR_SS2_POINTCLOUD[] = "/sensor/ss2/pointcloud";

	static constexpr char STATUS_CEIL[] = "/status/ceil";
	static constexpr char STATUS_CORR[] = "/status/corr";
	static constexpr char STATUS_LIDAR[] = "/status/lidar";
	static constexpr char STATUS_SS1[] = "/status/ss1";
	static constexpr char STATUS_SS2[] = "/status/ss2";

	static const char *addBodySuffix(const char *topic);
	static const char *addBodyLevelSuffix(const char *topic);
	static const char *addLatencySuffix(const char *topic);
	static const char *addDebugPrefix(const char *topic);
};

}

#endif
