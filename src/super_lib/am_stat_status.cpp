#include <am_utils/am_ros2_utility.h>
#include <super_lib/am_stat_status.h>

namespace am
{
  AMStatStatus::AMStatStatus(const std::string& short_name, const std::string& long_name) : AMStat(short_name, long_name)
  {
  }


  LifeCycleStatus AMStatStatus::process(double warn_throttle_s, double error_throttle_s)
  {
    LifeCycleStatus status = LifeCycleStatus::OK;
  
    if(!sample_required_ || sample_received_)
    {
      if((LifeCycleStatus)value_ == LifeCycleStatus::WARN)
      {
        ROS_WARN_STREAM_THROTTLE(warn_throttle_s, "Stats: " << long_name_ << " warning");
        compoundStatus(status, LifeCycleStatus::WARN);
      }
      else if((LifeCycleStatus)value_ == LifeCycleStatus::ERROR)
      {
        ROS_ERROR_STREAM_THROTTLE(warn_throttle_s, "Stats: " << long_name_ << " error");
        compoundStatus(status, LifeCycleStatus::ERROR);
      }
    }
    else
    {
      //sample is required and not yet received
      status = LifeCycleStatus::ERROR;
    }
    return status;
  }
  
  AMStatStatus& AMStatStatus::operator=(uint32_t assignment)
  {
    sample_received_ = true;
    value_ = assignment;
    return *this;
  }

};// namespace am

