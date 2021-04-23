#ifndef AM_UTILS_INCLUDE_AM_UTILS_AM_PCL_UTILS_H_
#define AM_UTILS_INCLUDE_AM_UTILS_AM_PCL_UTILS_H_

#include <sensor_msgs/PointCloud2.h>
#include <am_utils/point_types.h>

namespace am
{

class AMPCLUtils
{

public:
  static pcl::PointCloud<pcl::PointXYZIR>::ConstPtr convertToPointXYZIR(
      const sensor_msgs::PointCloud2::ConstPtr pc2,
      pcl::PointCloud<pcl::PointXYZIR>::Ptr &pcl);
};

}

#endif /* AM_UTILS_INCLUDE_AM_UTILS_AM_PCL_UTILS_H_ */
