#ifndef AM_UTILS_INCLUDE_AM_UTILS_AM_PCL_UTILS_H_
#define AM_UTILS_INCLUDE_AM_UTILS_AM_PCL_UTILS_H_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <am_utils/point_types.h>
#include <unordered_set>

namespace am
{

class AMPCLUtils
{

public:
  static pcl::PointCloud<pcl::PointXYZIR>::ConstPtr convertToPointXYZIR(const sensor_msgs::msg::PointCloud2::SharedPtr pc2,
      pcl::PointCloud<pcl::PointXYZIR>::Ptr &pcl);

  static pcl::PointCloud<pcl::PointXYZ>::ConstPtr convertToPointXYZIR(const sensor_msgs::msg::PointCloud2::SharedPtr pc2,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl);

  static void convertToPointXYZIR(const sensor_msgs::msg::PointCloud2::SharedPtr pc2,
        pcl::PointCloud<pcl::PointXYZIR> &pcl);

  static void convertToPointXYZIR(const sensor_msgs::msg::PointCloud2 &pc2,
          pcl::PointCloud<pcl::PointXYZIR> &pcl);

  static void convertToPointXYZ(const sensor_msgs::msg::PointCloud2::SharedPtr pc2,
         pcl::PointCloud<pcl::PointXYZ> &pcl);

  static void convertToPointXYZ(const sensor_msgs::msg::PointCloud2 &pc2,
           pcl::PointCloud<pcl::PointXYZ> &pcl);

  static void toPointCloud2(const pcl::PointCloud<pcl::PointXYZ> &cloud, sensor_msgs::msg::PointCloud2 &cloud_out);

  static void toPointCloud2(const pcl::PointCloud<pcl::PointXYZIR> &cloud, sensor_msgs::msg::PointCloud2 &cloud_out);

  static void toPointCloud2(const pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud,
		  const std::unordered_set<int> &valid_indices, sensor_msgs::msg::PointCloud2 &cloud_out);

  static bool isSetToNaN(pcl::PointXYZIR &point);

  static void setNaN(pcl::PointXYZIR &point);
};

}

#endif /* AM_UTILS_INCLUDE_AM_UTILS_AM_PCL_UTILS_H_ */
