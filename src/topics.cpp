#include <string>
#include <am_utils/topics.h>

using namespace am;

constexpr char am_topics::DJI_SDK_IMU[];
constexpr char am_topics::DJI_SDK_RTK_POSITION[];

constexpr char am_topics::FEATURE_ODOMETRY[];
constexpr char am_topics::FEATURE_SEARCH_IDS[];

constexpr char am_topics::SENSOR_LIDAR_SCAN[];
constexpr char am_topics::SENSOR_LIDAR_SCAN_FILTERED[];
constexpr char am_topics::SENSOR_LIDAR_POINTCLOUD[];
constexpr char am_topics::SENSOR_LIDAR_POINTCLOUD2[];

constexpr char am_topics::SENSOR_OUSTER_POINTCLOUD2[];

constexpr char am_topics::SENSOR_SS1_DEPTH_IMAGE_RECT[];
constexpr char am_topics::SENSOR_SS1_DEPTH_CAMERA_INFO[];
constexpr char am_topics::SENSOR_SS1_POINTCLOUD[];
constexpr char am_topics::SENSOR_SS1_POINTCLOUD2[];
constexpr char am_topics::SENSOR_SS1_PCL[];
constexpr char am_topics::SENSOR_SS2_DEPTH_IMAGE_RECT[];
constexpr char am_topics::SENSOR_SS2_DEPTH_CAMERA_INFO[];
constexpr char am_topics::SENSOR_SS2_POINTCLOUD[];
constexpr char am_topics::SENSOR_SS2_POINTCLOUD2[];
constexpr char am_topics::SENSOR_SS2_PCL[];

constexpr char am_topics::SENSOR_SC1_DEPTH_IMAGE_RECT[];
constexpr char am_topics::SENSOR_SC1_DEPTH_CAMERA_INFO[];
constexpr char am_topics::SENSOR_SC1_IMU[];
constexpr char am_topics::SENSOR_SC1_IR_IMAGE[];
constexpr char am_topics::SENSOR_SC1_IR_CAMERA_INFO[];
constexpr char am_topics::SENSOR_SC1_POINTCLOUD[];
constexpr char am_topics::SENSOR_SC1_VISIBLE_IMAGE[];
constexpr char am_topics::SENSOR_SC1_VISIBLE_CAMERA_INFO[];

constexpr char am_topics::SENSOR_SC2_DEPTH_IMAGE_RECT[];
constexpr char am_topics::SENSOR_SC2_DEPTH_CAMERA_INFO[];
constexpr char am_topics::SENSOR_SC2_IMU[];
constexpr char am_topics::SENSOR_SC2_IR_IMAGE[];
constexpr char am_topics::SENSOR_SC2_IR_CAMERA_INFO[];
constexpr char am_topics::SENSOR_SC2_POINTCLOUD[];
constexpr char am_topics::SENSOR_SC2_VISIBLE_IMAGE[];
constexpr char am_topics::SENSOR_SC2_VISIBLE_CAMERA_INFO[];

constexpr char am_topics::SENSOR_SC3_DEPTH_IMAGE_RECT[];
constexpr char am_topics::SENSOR_SC3_DEPTH_CAMERA_INFO[];
constexpr char am_topics::SENSOR_SC3_IMU[];
constexpr char am_topics::SENSOR_SC3_IR_IMAGE[];
constexpr char am_topics::SENSOR_SC3_IR_CAMERA_INFO[];
constexpr char am_topics::SENSOR_SC3_POINTCLOUD[];
constexpr char am_topics::SENSOR_SC3_VISIBLE_IMAGE[];
constexpr char am_topics::SENSOR_SC3_VISIBLE_CAMERA_INFO[];

constexpr char am_topics::SENSOR_DISTANCE_AGL[];
constexpr char am_topics::SENSOR_DISTANCE_AGL_LW[];
constexpr char am_topics::SENSOR_DISTANCE_DISTANCES_LW20[];

constexpr char am_topics::STATUS_CEIL[];
constexpr char am_topics::STATUS_CORR[];
constexpr char am_topics::STATUS_LIDAR[];
constexpr char am_topics::STATUS_SS1[];
constexpr char am_topics::STATUS_SS2[];

constexpr char am_topics::LED_BLINK[];
constexpr char am_topics::BATTERY_STATE[];

constexpr char am_topics::SENSOR_CAMERA_240P_RAW_FRONT_LEFT[];
constexpr char am_topics::SENSOR_CAMERA_240P_RAW_FRONT_RIGHT[];
constexpr char am_topics::SENSOR_CAMERA_240P_RAW_DOWN_FRONT[];
constexpr char am_topics::SENSOR_CAMERA_240P_RAW_DOWN_BACK[];

constexpr char am_topics::SENSOR_CAMERA_240P_RECT_FRONT_LEFT[];
constexpr char am_topics::SENSOR_CAMERA_240P_RECT_FRONT_RIGHT[];
constexpr char am_topics::SENSOR_CAMERA_240P_RECT_DOWN_FRONT[];
constexpr char am_topics::SENSOR_CAMERA_240P_RECT_DOWN_BACK[];

constexpr char am_topics::SENSOR_CAMERA_FPV_MAIN[];


std::string am_topics::addBodySuffix(const char *topic)
{
	std::string latency_topic = std::string(topic) + "/body_FLU";
	return latency_topic;
}

std::string am_topics::addBodySuffix(std::string topic)
{
	return am_topics::addBodySuffix(topic.c_str());
}

std::string am_topics::addBodyLevelSuffix(const char *topic)
{
	std::string latency_topic = std::string(topic) + "/body_Level_FLU";
	return latency_topic;
}

std::string am_topics::addBodyLevelSuffix(std::string topic)
{
	return am_topics::addBodyLevelSuffix(topic.c_str());
}

std::string am_topics::addLatencySuffix(const char *topic)
{
	std::string latency_topic = std::string(topic) + "_latency";
	return latency_topic;
}

std::string am_topics::addLatencySuffix(std::string topic)
{
	return am_topics::addLatencySuffix(topic.c_str());
}

std::string am_topics::addDebugPrefix(const char *topic)
{
	std::string debug_topic = "/debug" + std::string(topic);
	return debug_topic;
}

std::string am_topics::addDebugPrefix(std::string topic)
{
	return am_topics::addDebugPrefix(topic.c_str());
}

