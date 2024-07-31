#include <am_utils/am_ros2_utility.h>
#include <super_lib/am_stat_ave.h>

namespace am
{
  AMStatAve::AMStatAve(const std::string& short_name, const std::string& long_name) : AMStatReset(short_name, long_name)
  {
  }

  AMStatAve::AMStatAve(const std::string& short_name, const std::string& long_name, uint32_t max_warn, uint32_t max_error)
    : AMStatReset(short_name, long_name, max_warn, max_error)
  {
  }

  AMStatAve::AMStatAve(const std::string& short_name, const std::string& long_name, uint32_t min_error, uint32_t min_warn,
            uint32_t max_warn, uint32_t max_error, uint32_t min_min_error, uint32_t min_min_warn,
            uint32_t max_max_warn, uint32_t max_max_error)
    : AMStatReset(short_name, long_name, min_error, min_warn, max_warn, max_error)
  {
    min_min_error_ = min_min_error;
    min_min_warn_ = min_min_warn;
    max_max_warn_ = max_max_warn;
    max_max_error_ = max_max_error;
  }

  LifeCycleStatus AMStatAve::process(double warn_throttle_s, double error_throttle_s) 
  {
    uint32_t ave = getAve();
    LifeCycleStatus status = LifeCycleStatus::OK;

    if (ave > max_error_)
    {
      ROS_ERROR_STREAM_THROTTLE(error_throttle_s, "stat: " << long_name_ << "ave exceeding max_error: " << ave
                                                             << " (max:" << max_error_ << ")");
      compoundStatus(status, LifeCycleStatus::ERROR);
    }
    else if (ave > max_warn_)
    {
      ROS_WARN_STREAM_THROTTLE(warn_throttle_s, "stat: " << long_name_ << " ave exceeding max_warn: " << ave
                                                           << " (max:" << max_warn_ << ")");
      compoundStatus(status, LifeCycleStatus::WARN);
    }

    if (ave < min_error_)
    {
      ROS_ERROR_STREAM_THROTTLE(error_throttle_s, "stat: " << long_name_ << " ave exceeding min_error: " << ave
                                                             << " (min:" << min_error_ << ")");
      compoundStatus(status, LifeCycleStatus::ERROR);
    }
    else if (ave < min_warn_)
    {
      ROS_WARN_STREAM_THROTTLE(warn_throttle_s, "stat: " << long_name_ << " ave exceeding min_warn: " << ave
                                                           << " (min:" << min_warn_ << ")");
      compoundStatus(status, LifeCycleStatus::WARN);
    }

    if (min_ < min_min_error_)
    {
      ROS_ERROR_STREAM_THROTTLE(error_throttle_s, "stat: " << long_name_ << " min exceeding min_min_error: " << min_
                                                             << " (min:" << min_min_error_ << ")");
      compoundStatus(status, LifeCycleStatus::ERROR);
    }
    else if (min_ < min_min_warn_)
    {
      ROS_WARN_STREAM_THROTTLE(warn_throttle_s, "stat: " << long_name_ << " min exceeding min_min_warn: " << min_
                                                           << " (min:" << min_min_warn_ << ")");
      compoundStatus(status, LifeCycleStatus::WARN);
    }

    if (max_ > max_max_error_)
    {
      ROS_ERROR_STREAM_THROTTLE(error_throttle_s, "stat: " << long_name_ << " max exceeding max_max_error: " << max_
                                                             << " (max:" << max_max_error_ << ")");
      compoundStatus(status, LifeCycleStatus::ERROR);
    }
    else if (max_ > max_max_warn_)
    {
      ROS_WARN_STREAM_THROTTLE(warn_throttle_s, "stat: " << long_name_ << " max exceeding max_max_warn: " << max_
                                                           << " (max:" << max_max_warn_ << ")");
      compoundStatus(status, LifeCycleStatus::WARN);
    }

    return status;
  }

  void AMStatAve::reset()
  {
    value_ = 0;
    total_ = 0;
    max_ = 0;
    min_ = std::numeric_limits<uint32_t>::max();
  }

  void AMStatAve::add(uint32_t value)
  {
    total_ += value;
    value_++;
    if (value < min_)
    {
      min_ = value;
    }
    if (value > max_)
    {
      max_ = value;
    }
  }

  uint32_t AMStatAve::getAve()
  {
    if(value_ == 0)
    {
      return 0;
    }
    
    uint64_t ave_64 = ((float)total_ / (float)value_ + 0.5);
    uint32_t ave_32 = ave_64 > std::numeric_limits<uint32_t>::max() ? std::numeric_limits<uint32_t>::max() : ave_64;
    return ave_32;
  }

  void AMStatAve::addStatistics(diagnostic_updater::DiagnosticStatusWrapper& dsw) 
  {
    dsw.add(long_name_ + " Ave", getAve());
    dsw.add(long_name_ + " Max", getMax());
    dsw.add(long_name_ + " Min", getMin());
  }

  std::string AMStatAve::getStrShort() 
  {
    std::stringstream ss;
    if(!total_)
    {
      ss << short_name_ << "-av:0," << short_name_ << "-mx:0," << short_name_ << "-mn:0";
    }
    else
    {
      ss << short_name_ << "-av:" << getAve() << "," << short_name_ << "-mx:" << getMax() << "," << short_name_
        << "-mn:" << getMin();
    }
    return ss.str();
  }

  std::string AMStatAve::getStr() 
  {
    std::stringstream ss;
    if(!total_)
    {
      ss << long_name_ << " Ave: 0," << long_name_ << " Max: 0," << long_name_ << " Min: 0";
    }
    else
    {
      ss << long_name_ << " Ave: " << getAve() << "," << long_name_ << " Max: " << getMax() << "," << long_name_
        << " Min: " << getMin();
    }
    return ss.str();
  }

  uint32_t AMStatAve::getMax() const
  {
    return max_;
  }

  uint32_t AMStatAve::getMaxMaxError() const
  {
    return max_max_error_;
  }

  uint32_t AMStatAve::getMaxMaxWarn() const
  {
    return max_max_warn_;
  }

  uint32_t AMStatAve::getMin() const
  {
    return min_;
  }

  uint32_t AMStatAve::getMinMinError() const
  {
    return min_min_error_;
  }

  uint32_t AMStatAve::getMinMinWarn() const
  {
    return min_min_warn_;
  }

  uint64_t AMStatAve::getTotal() const
  {
    return total_;
  }

  void AMStatAve::setMax(uint32_t max)
  {
    max_ = max;
  }

  void AMStatAve::setMaxMaxError(uint32_t maxMaxError)
  {
    max_max_error_ = maxMaxError;
  }

  void AMStatAve::setMaxMaxWarn(uint32_t maxMaxWarn)
  {
    max_max_warn_ = maxMaxWarn;
  }

  void AMStatAve::setMin(uint32_t min)
  {
    min_ = min;
  }

  void AMStatAve::setMinMinError(uint32_t minMinError)
  {
    min_min_error_ = minMinError;
  }

  void AMStatAve::setMinMinWarn(uint32_t minMinWarn)
  {
    min_min_warn_ = minMinWarn;
  }
}; // namespace am


