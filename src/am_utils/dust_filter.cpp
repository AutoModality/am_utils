#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <am_utils/dust_filter.h>
#include <am_utils/point_types.h>
#include <vb_util_lib/frame_names.h>
#include <vb_util_lib/topics.h>

#define DEFAULT_RANGE_THRESHHOLD_M (0.1)
#define DEFAULT_MIN_NEIGHBORS      (2)
#define DEFAULT_MAX_RANGE_M        (7.5)

namespace am
{

DustFilter::DustFilter(ros::NodeHandle &nh, std::string name_space) : nh_(nh), name_space_(name_space)
{
  getParams();

  debug_pcl_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZIR>>("/debug/dust_filter/pcl", 10);
}

DustFilter::~DustFilter()
{
}

void DustFilter::getParams()
{
  static bool first_run = true;

  nh_.param<float>(name_space_ + "/range_threshold_m", range_threshold_m_, DEFAULT_RANGE_THRESHHOLD_M);
  nh_.param<int>(name_space_ + "/min_neighbors", min_neighbors_, DEFAULT_MIN_NEIGHBORS);
  nh_.param<float>(name_space_ + "/max_range_m", max_range_m_, DEFAULT_MAX_RANGE_M);
  nh_.param<float>(name_space_ + "/inside_min_x_m", inside_min_x_m_, -0.5);
  nh_.param<float>(name_space_ + "/inside_max_x_m", inside_max_x_m_, 0.5);
  nh_.param<float>(name_space_ + "/inside_min_y_m", inside_min_y_m_, -0.5);
  nh_.param<float>(name_space_ + "/inside_max_y_m", inside_max_y_m_, 0.5);
  nh_.param<float>(name_space_ + "/inside_min_z_m", inside_min_z_m_, -0.5);
  nh_.param<float>(name_space_ + "/inside_max_z_m", inside_max_z_m_, 0.5);
  nh_.param<float>(name_space_ + "/outside_min_x_m", outside_min_x_m_, std::numeric_limits<float>::lowest());
  nh_.param<float>(name_space_ + "/outside_max_x_m", outside_max_x_m_, std::numeric_limits<float>::max());
  nh_.param<float>(name_space_ + "/outside_min_y_m", outside_min_y_m_, std::numeric_limits<float>::lowest());
  nh_.param<float>(name_space_ + "/outside_max_y_m", outside_max_y_m_, std::numeric_limits<float>::max());
  nh_.param<float>(name_space_ + "/outside_min_z_m", outside_min_z_m_, std::numeric_limits<float>::lowest());
  nh_.param<float>(name_space_ + "/outside_max_z_m", outside_max_z_m_, std::numeric_limits<float>::max());


  if (first_run)
  {
    first_run = false;
    ROS_INFO_STREAM(name_space_ + "/range_threshold_m = " << range_threshold_m_);
    ROS_INFO_STREAM(name_space_ + "/neighbors = " << min_neighbors_);
    printInsideParams();
    printOutsideParams();
  }
}

void DustFilter::printInsideParams()
{
  ROS_INFO_STREAM(name_space_ + "/inside_max_x_m = " << inside_max_x_m_);
  ROS_INFO_STREAM(name_space_ + "/inside_max_x_m = " << inside_max_x_m_);
  ROS_INFO_STREAM(name_space_ + "/inside_min_y_m = " << inside_min_y_m_);
  ROS_INFO_STREAM(name_space_ + "/inside_max_y_m = " << inside_max_y_m_);
  ROS_INFO_STREAM(name_space_ + "/inside_min_z_m = " << inside_min_z_m_);
  ROS_INFO_STREAM(name_space_ + "/inside_max_z_m = " << inside_max_z_m_);
}

void DustFilter::printOutsideParams()
{
  ROS_INFO_STREAM(name_space_ + "/outside_min_x_m = " << outside_min_x_m_);
  ROS_INFO_STREAM(name_space_ + "/outside_max_x_m = " << outside_max_x_m_);
  ROS_INFO_STREAM(name_space_ + "/outside_min_y_m = " << outside_min_y_m_);
  ROS_INFO_STREAM(name_space_ + "/outside_max_y_m = " << outside_max_y_m_);
  ROS_INFO_STREAM(name_space_ + "/outside_min_z_m = " << outside_min_z_m_);
  ROS_INFO_STREAM(name_space_ + "/outside_max_z_m = " << outside_max_z_m_);
}

int DustFilter::process(pcl::PointCloud<pcl::PointXYZIR> &pcl)
{
  bool nan_mask[pcl.width * pcl.width];

  bool is_dense = pcl.is_dense;
  for (uint32_t width = 0; width < pcl.width; width++)
  {
    for (uint32_t height = 0; height < pcl.height; height++)
    {
      int index = width * pcl.height + height;
      pcl::PointXYZIR &point = pcl.points[index];

      // if already nan, continue
      if(isSetToNaN(point))
      {
        nan_mask[index] = true;
        continue;
      }

      // if beyond the max range, don't filter dust
      bool skip_dust_filter = false;
      if(point.range > max_range_m_)
      {
        skip_dust_filter = true;
      }

      // filter anything close to the center
      if(point.x < inside_max_x_m_ && point.x > inside_min_x_m_ &&
          point.y < inside_max_y_m_ && point.y > inside_min_y_m_ &&
          point.z < inside_max_z_m_ && point.z > inside_min_z_m_)
      {
        nan_mask[index] = true;
        continue;
      }

      // filter anything too far away
      if(point.x > outside_max_x_m_ || point.x < outside_min_x_m_ ||
          point.y > outside_max_y_m_ || point.y < outside_min_y_m_ ||
          point.z > outside_max_z_m_ || point.z < outside_min_z_m_)
      {
        nan_mask[index] = true;
        continue;
      }

      if(!skip_dust_filter)
      {
        // filter points without neighbors
        // width wraps, height doesn't
        int neighbor_width_start = width - 1;
        neighbor_width_start = neighbor_width_start < 0 ? pcl.width + neighbor_width_start : neighbor_width_start;
        int neighbor_width_end = width + 2;
        neighbor_width_end = neighbor_width_end > pcl.width - 1 ? neighbor_width_end - pcl.width : neighbor_width_end;
        int neighbor_height_start = height == 0 ? height : height - 1;
        int neighbor_height_end = height == pcl.height - 1 ? height : height + 1;
        int neighbors = 0;
        int nw = neighbor_width_start;
        while(nw != neighbor_width_end)
        {
          for(int nh = neighbor_height_start; nh <= neighbor_height_end; nh++)
          {
            // don't compare to itself
            if(nh == height && nw == width)
            {
              continue;
            }
            int neighbor_index = nw * pcl.height + nh;
            pcl::PointXYZIR &neighbor_point = pcl.points[neighbor_index];
            if(std::abs(neighbor_point.range - point.range) < range_threshold_m_)
            {
              neighbors++;
            }
          }
          nw++;
          nw = nw > pcl.width - 1 ? nw - pcl.width : nw;
        }
        if(neighbors < min_neighbors_)
        {
          nan_mask[index] = true;
          is_dense = false;
        }
        else
        {
          nan_mask[index] = false;
        }
      }
    }

  }

  int points_removed = 0;
  for (uint32_t width = 0; width < pcl.width; width++)
  {
    for (uint32_t height = 0; height < pcl.height; height++)
    {
      int index = width * pcl.height + height;
      pcl::PointXYZIR &point = pcl.points[index];
      if(nan_mask[index])
      {
        setNaN(point);
        points_removed++;
        is_dense = false;
      }
    }
  }

  pcl.is_dense = is_dense;

  if (debug_pcl_pub_.getNumSubscribers() > 0)
  {
    debug_pcl_pub_.publish(pcl);
  }

  return points_removed;
}

bool DustFilter::isSetToNaN(pcl::PointXYZIR &point)
{
  if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) ||
      std::isnan(point.intensity) || std::isnan(point.intensity))
  {
    setNaN(point);
    return true;
  }
  return false;
}

void DustFilter::setNaN(pcl::PointXYZIR &point)
{
  point.x = std::nanf("");
  point.y = std::nanf("");
  point.z = std::nanf("");
  point.intensity = std::nanf("");
  point.range = std::nanf("");
}

void DustFilter::setInsideFilter(float min_x_m_, float max_x_m_, float min_y_m_, float max_y_m_, float min_z_m_, float max_z_m_)
{
  inside_min_x_m_ = min_x_m_;
  inside_max_x_m_ = max_x_m_;
  inside_min_y_m_ = min_y_m_;
  inside_max_y_m_ = max_y_m_;
  inside_min_z_m_ = min_z_m_;
  inside_max_z_m_ = max_z_m_;
}

void DustFilter::setOutsideFilter(float min_x_m_, float max_x_m_, float min_y_m_, float max_y_m_, float min_z_m_, float max_z_m_)
{
  outside_min_x_m_ = min_x_m_;
  outside_max_x_m_ = max_x_m_;
  outside_min_y_m_ = min_y_m_;
  outside_max_y_m_ = max_y_m_;
  outside_min_z_m_ = min_z_m_;
  outside_max_z_m_ = max_z_m_;
}

} // namespace

