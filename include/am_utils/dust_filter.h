#ifndef AM_UTILS_INCLUDE_AM_UTILS_DUST_FILTER_H_
#define AM_UTILS_INCLUDE_AM_UTILS_DUST_FILTER_H_

#include <pcl_ros/point_cloud.h>

#include <am_utils/point_types.h>

namespace am
{

class DustFilter
{
private:

  ros::NodeHandle nh_;

  ros::Publisher debug_pcl_pub_;

  float range_threshold_m_;
  float max_range_m_;
  int min_neighbors_;
  std::string name_space_;

  float inside_min_x_m_;
  float inside_max_x_m_;
  float inside_min_y_m_;
  float inside_max_y_m_;
  float inside_min_z_m_;
  float inside_max_z_m_;
  float outside_min_x_m_;
  float outside_max_x_m_;
  float outside_min_y_m_;
  float outside_max_y_m_;
  float outside_min_z_m_;
  float outside_max_z_m_;

  bool isSetToNaN(pcl::PointXYZIR &point);
  void setNaN(pcl::PointXYZIR &point);
  void printInsideParams();
  void printOutsideParams();

public:
  DustFilter(ros::NodeHandle &nh, std::string name_space);
  ~DustFilter();

  int process(pcl::PointCloud<pcl::PointXYZIR> &pc);
  void getParams();
  void setInsideFilter(float min_x_m_, float max_x_m_, float min_y_m_, float max_y_m_, float min_z_m_, float max_z_m_);
  void setOutsideFilter(float min_x_m_, float max_x_m_, float min_y_m_, float max_y_m_, float min_z_m_, float max_z_m_);
};

}

#endif /* AM_UTILS_INCLUDE_AM_UTILS_DUST_FILTER_H_ */
