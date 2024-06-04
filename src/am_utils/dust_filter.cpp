#include <pcl_conversions/pcl_conversions.h>

#include <am_utils/am_pcl_utils.h>
#include <am_utils/dust_filter.h>
#include <am_utils/point_types.h>
#include <vb_util_lib/frame_names.h>
#include <vb_util_lib/topics.h>
#include <am_utils/am_ros2_utility.h>

float DEFAULT_RANGE_THRESHHOLD_M {1.0};
int DEFAULT_MIN_NEIGHBORS      {2};
float DEFAULT_MAX_RANGE_M        {7.5};

namespace am
{

DustFilter::DustFilter(std::string name_space) :name_space_(name_space)
{
  getParams();
  //debug_pcl_pub_ = this->create_publisher<pcl::PointCloud<pcl::PointXYZIR>>("/debug/" + "dust_filter/dust_filter/pcl", 10);
}

DustFilter::~DustFilter()
{
}

void DustFilter::getParams()
{
  static bool first_run = true;

  am::getParam<float>("dust_filter.range_threshold_m", range_threshold_m_, DEFAULT_RANGE_THRESHHOLD_M);
  am::getParam<int>("dust_filter.min_neighbors", min_neighbors_, DEFAULT_MIN_NEIGHBORS);
  am::getParam<float>("dust_filter.inside_min_x_m", inside_min_x_m_, static_cast<float>(-0.7));
  am::getParam<float>("dust_filter.inside_max_x_m", inside_max_x_m_, static_cast<float>(0.7));
  am::getParam<float>("dust_filter.inside_min_y_m", inside_min_y_m_, static_cast<float>(-0.7));
  am::getParam<float>("dust_filter.inside_max_y_m", inside_max_y_m_, static_cast<float>(0.7));
  am::getParam<float>("dust_filter.inside_min_z_m", inside_min_z_m_, static_cast<float>(-0.5));
  am::getParam<float>("dust_filter.inside_max_z_m", inside_max_z_m_, static_cast<float>(0.5));
  am::getParam<float>("dust_filter.outside_min_x_m", outside_min_x_m_, std::numeric_limits<float>::lowest());
  am::getParam<float>("dust_filter.outside_max_x_m", outside_max_x_m_, std::numeric_limits<float>::max());
  am::getParam<float>("dust_filter.outside_min_y_m", outside_min_y_m_, std::numeric_limits<float>::lowest());
  am::getParam<float>("dust_filter.outside_max_y_m", outside_max_y_m_, std::numeric_limits<float>::max());
  am::getParam<float>("dust_filter.outside_min_z_m", outside_min_z_m_, std::numeric_limits<float>::lowest());
  am::getParam<float>("dust_filter.outside_max_z_m", outside_max_z_m_, std::numeric_limits<float>::max());

  if (first_run)
  {
    first_run = false;
    ROS_INFO_STREAM("dust_filter/range_threshold_m = " << range_threshold_m_);
    ROS_INFO_STREAM("dust_filter/neighbors = " << min_neighbors_);
    printInsideParams();
    printOutsideParams();
  }
}

void DustFilter::printInsideParams()
{
	ROS_INFO_STREAM("dust_filter/inside_min_x_m = " << inside_min_x_m_);
	ROS_INFO_STREAM("dust_filter/inside_max_x_m = " << inside_max_x_m_);
	ROS_INFO_STREAM("dust_filter/inside_min_y_m = " << inside_min_y_m_);
	ROS_INFO_STREAM("dust_filter/inside_max_y_m = " << inside_max_y_m_);
	ROS_INFO_STREAM("dust_filter/inside_min_z_m = " << inside_min_z_m_);
	ROS_INFO_STREAM("dust_filter/inside_max_z_m = " << inside_max_z_m_);
}

void DustFilter::printOutsideParams()
{
	ROS_INFO_STREAM("dust_filter/outside_min_x_m = " << outside_min_x_m_);
	ROS_INFO_STREAM("dust_filter/outside_max_x_m = " << outside_max_x_m_);
	ROS_INFO_STREAM("dust_filter/outside_min_y_m = " << outside_min_y_m_);
	ROS_INFO_STREAM("dust_filter/outside_max_y_m = " << outside_max_y_m_);
	ROS_INFO_STREAM("dust_filter/outside_min_z_m = " << outside_min_z_m_);
	ROS_INFO_STREAM("dust_filter/outside_max_z_m = " << outside_max_z_m_);
}

int DustFilter::process(pcl::PointCloud<pcl::PointXYZIR> &pcl, bool skip_dust_filter)
{
  bool nan_mask[pcl.width * pcl.height];

  bool is_dense = pcl.is_dense;
  for (uint32_t width = 0; width < pcl.width; width++)
  {
    for (uint32_t height = 0; height < pcl.height; height++)
    {
      int index = width * pcl.height + height;
      nan_mask[index] = false;
      pcl::PointXYZIR &point = pcl.points[index];

      // if already nan, continue
      if(AMPCLUtils::isSetToNaN(point))
      {
        nan_mask[index] = true;
        continue;
      }

      // if beyond the max range, don't filter dust
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
        AMPCLUtils::setNaN(point);
        points_removed++;
        is_dense = false;
      }
    }
  }

  pcl.is_dense = is_dense;

  //if (debug_pcl_pub_->get_subscription_count() > 0)
  //{
    //debug_pcl_pub_->publish(pcl);
  //}

  return points_removed;
}


int DustFilter::process(pcl::PointCloud<pcl::PointXYZ> &pcl, bool skip_dust_filter)
{
  bool nan_mask[pcl.width * pcl.height];

  bool is_dense = pcl.is_dense;
  for (uint32_t width = 0; width < pcl.width; width++)
  {
    for (uint32_t height = 0; height < pcl.height; height++)
    {
      int index = width * pcl.height + height;
      nan_mask[index] = false;
      pcl::PointXYZ &point = pcl.points[index];

      // if already nan, continue
      if(AMPCLUtils::isSetToNaN(point))
      {
        nan_mask[index] = true;
        continue;
      }
      double range = sqrt(pow(point.x,2) + pow(point.y,2) + pow(point.z,2));
      // if beyond the max range, don't filter dust
      if(range > max_range_m_)
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
            pcl::PointXYZ &neighbor_point = pcl.points[neighbor_index];
            double neighbor_point_range = sqrt(pow(neighbor_point.x,2) + pow(neighbor_point.y,2) + pow(neighbor_point.z,2));
            if(std::abs(neighbor_point_range - range) < range_threshold_m_)
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
      pcl::PointXYZ &point = pcl.points[index];
      if(nan_mask[index])
      {
        AMPCLUtils::setNaN(point);
        points_removed++;
        is_dense = false;
      }
    }
  }

  pcl.is_dense = is_dense;

  //if (debug_pcl_pub_->get_subscription_count() > 0)
  //{
    //debug_pcl_pub_->publish(pcl);
  //}

  return points_removed;
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

