#include <ros/ros.h>

#include <am_utils/am_pcl_utils.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>

namespace am
{

pcl::PointCloud<pcl::PointXYZIR>::ConstPtr AMPCLUtils::convertToPointXYZIR(
    const sensor_msgs::PointCloud2::ConstPtr pc2, pcl::PointCloud<pcl::PointXYZIR>::Ptr &pcl)
{
  pcl->header = pcl_conversions::toPCL(pc2->header);
  pcl->resize(pc2->height * pc2->width);
  pcl->height = pc2->height;
  pcl->width = pc2->width;
  int index = 0;
  for (sensor_msgs::PointCloud2ConstIterator<float> it(*pc2, "x"); it != it.end(); ++it) {
      float x = it[0];
      float y = it[1];
      float z = it[2];
      float intensity = it[4];
      pcl->points[index].x = x;
      pcl->points[index].y = y;
      pcl->points[index].z = z;
      pcl->points[index].intensity = intensity;
      pcl->points[index].range = std::sqrt(x * x + y * y + z * z);
      index++;
  }

  return pcl;
}

bool AMPCLUtils::isSetToNaN(pcl::PointXYZIR &point)
{
  if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) ||
      std::isnan(point.intensity) || std::isnan(point.intensity))
  {
    setNaN(point);
    return true;
  }
  return false;
}

void AMPCLUtils::setNaN(pcl::PointXYZIR &point)
{
  point.x = std::nanf("");
  point.y = std::nanf("");
  point.z = std::nanf("");
  point.intensity = std::nanf("");
  point.range = std::nanf("");
}

} // namespace

