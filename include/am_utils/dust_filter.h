#ifndef AM_UTILS_INCLUDE_AM_UTILS_DUST_FILTER_H_
#define AM_UTILS_INCLUDE_AM_UTILS_DUST_FILTER_H_

#include <rclcpp/rclcpp.hpp>
#include <am_utils/point_types.h>

namespace am
{

class DustFilter : public rclcpp::Node
{
private:
  //rclcpp::Publisher<pcl::PointCloud<pcl::PointXYZIR>>::SharedPtr debug_pcl_pub_;

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

  void printInsideParams();
  void printOutsideParams();



public:
  DustFilter(const std::string &node_name, std::string name_space);
  ~DustFilter();

  int process(pcl::PointCloud<pcl::PointXYZIR> &pc, bool skip_dust_filter = false);
  void getParams();
  void setInsideFilter(float min_x_m_, float max_x_m_, float min_y_m_, float max_y_m_, float min_z_m_, float max_z_m_);
  void setOutsideFilter(float min_x_m_, float max_x_m_, float min_y_m_, float max_y_m_, float min_z_m_, float max_z_m_);
};

}

#endif /* AM_UTILS_INCLUDE_AM_UTILS_DUST_FILTER_H_ */
