#ifndef AM_UTILS_AM_TOPICS_H
#define AM_UTILS_AM_TOPICS_H

namespace am
{

class am_topics
{
public:
	static constexpr char FEATURE_ODOMETRY[] = "/feature/odometry";
	static constexpr char LIDAR_SCAN[] = "/sensor/lidar/scan";
	static constexpr char LIDAR_POINTCLOUD[] = "/sensor/lidar/pointcloud";

	static std::string addBodySuffix(const char *topic);
	static std::string addBodyLevelSuffix(const char *topic);
	static std::string addLatencySuffix(const char *topic);
	static std::string addDebugPrefix(const char *topic);
};

}

#endif
