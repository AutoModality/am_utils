#include <string>
#include <am_utils/topics.h>

using namespace am;

constexpr char am_topics::FEATURE_ODOMETRY[];
constexpr char am_topics::LIDAR_SCAN[];
constexpr char am_topics::LIDAR_POINTCLOUD[];

std::string am_topics::addBodySuffix(const char *topic)
{
	std::string latency_topic = std::string(topic) + "/body_FLU";
	return latency_topic;
}

std::string am_topics::addBodyLevelSuffix(const char *topic)
{
	std::string latency_topic = std::string(topic) + "/body_Level_FLU";
	return latency_topic;
}

std::string am_topics::addLatencySuffix(const char *topic)
{
	std::string latency_topic = std::string(topic) + "_latency";
	return latency_topic;
}

std::string am_topics::addDebugPrefix(const char *topic)
{
	std::string debug_topic = "/debug" + std::string(topic);
	return debug_topic;
}
