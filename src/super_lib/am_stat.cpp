#include <am_utils/am_ros2_utility.h>
#include <super_lib/am_stat.h>


namespace am
{
// static constexpr std::string AMLifeCycle::STATE_INVALID_STRING;
// static constexpr std::string AMLifeCycle::STATE_UNCONFIGURED_STRING;

  AMStat::AMStat(const std::string& short_name, const std::string& long_name)
  {
    short_name_ = short_name;
    long_name_ = long_name;
  }

  AMStat::AMStat( const std::string& short_name, const std::string& long_name, uint32_t max_warn, uint32_t max_error)
    : AMStat(short_name, long_name)
  {
    setMaxWarn(max_warn);
    setMaxError(max_error);
  }

  AMStat::AMStat(const std::string& short_name, const std::string& long_name, uint32_t min_error, uint32_t min_warn,
              uint32_t max_warn, uint32_t max_error)
      : AMStat(short_name, long_name,max_warn,max_error)
  {
    setMinError(min_error);
    setMinWarn(min_warn);
  } 

  AMStat::~AMStat()
  {
  }

  LifeCycleStatus AMStat::process(double warn_throttle_s, double error_throttle_s)
  {
    LifeCycleStatus status = LifeCycleStatus::OK;
  
    if(!sample_required_ || sample_received_)
    {
      if(validate_max_)
      {
        if (value_ > max_error_)
        {
         ROS_ERROR_STREAM_THROTTLE(error_throttle_s, "stat error: " << long_name_ << " exceeding max_error: " << value_
                                                               << " (max:" << max_error_ << ") [TF5R]");
          compoundStatus(status, LifeCycleStatus::ERROR);
        }
        else if (value_ > max_warn_)
        {
         ROS_WARN_STREAM_THROTTLE(warn_throttle_s, "stat warn: " << long_name_ << " exceeding max_warn: " << value_ << " (max:" << max_warn_
                                                             << ") [PO9P]");
          compoundStatus(status, LifeCycleStatus::WARN);
        }
      }

      if(validate_min_)
      {
        if (value_ < min_error_)
        {
         ROS_ERROR_STREAM_THROTTLE(error_throttle_s, "stat error: " << long_name_ << " exceeding min_error: " << value_
                                                               << " (min:" << min_error_ << ") [K08K]");
          compoundStatus(status, LifeCycleStatus::ERROR);
        }
        else if (value_ < min_warn_)
        {
         ROS_WARN_STREAM_THROTTLE(warn_throttle_s, "stat warn: " << long_name_ << " exceeding min_warn: " << value_ << " (min:" << min_warn_
                                                             << ") [H9H8]");
          compoundStatus(status, LifeCycleStatus::WARN);
        }
      }

      if(!validate_max_ && !validate_min_)
      {   
          //report this warning once during first validation
         ROS_WARN_STREAM_THROTTLE(9999, "stat warn: " << long_name_ << " lacks validation since min/max is not set [TRB5]");
      }
    }
    else
    {
      //sample is required and not yet received
      status = LifeCycleStatus::ERROR;
     ROS_ERROR_STREAM_THROTTLE(error_throttle_s, "stat error: " << long_name_ << " no samples received [NAQE] ");
    }
    return status;
  }

  void AMStat::reset()
  {
  }

 void AMStat::addStatistics(diagnostic_updater::DiagnosticStatusWrapper& dsw)
  {
    dsw.add(long_name_, value_);
  }

 std::string AMStat::getStrShort()
  {
    std::stringstream ss;
    ss << short_name_ << ":" << value_;
    return ss.str();
  }

 std::string AMStat::getStr()
  {
    std::stringstream ss;
    ss << long_name_ << ": " << value_;
    return ss.str();
  }

 void AMStat::add(uint32_t adder)
  {
    value_ += adder;
    sample_received_ = true;
  }

  AMStat& AMStat::operator++(int)
  {
    value_++;
    sample_received_ = true;
    return *this;
  }

  AMStat& AMStat::operator+=(int adder)
  {
    value_ += adder;
    sample_received_ = true;
    return *this;
  }

  AMStat& AMStat::operator=(uint32_t assignment)
  {
    value_ = assignment;
    sample_received_ = true;
    return *this;
  }

  /** Confusing name for return the value.  Use getValue() instead*/
  [[deprecated]]
  uint32_t AMStat::getCount() const
  {
    return value_;
  }

  /**The current value for the stat */
  uint32_t AMStat::getValue() const
  {
    return value_;
  }

  const std::string& AMStat::getLongName() const
  {
    return long_name_;
  }

  void AMStat::setWarnError(uint32_t max_warn, uint32_t max_error)
  {
    setMaxWarn(max_warn);
    setMaxError(max_error);
  }

  uint32_t AMStat::getMaxError() const
  {
    return max_error_;
  }

  void AMStat::setMaxError(uint32_t max_error)
  {
    max_error_ = max_error;
    validate_max_=true;
  }

  uint32_t AMStat::getMaxWarn() const
  {
    return max_warn_;
  }

  void AMStat::setMaxWarn(uint32_t max_warn)
  {
    validate_max_=true;
    max_warn_ = max_warn;
  }

  void AMStat::setWarnError(uint32_t min_error, uint32_t min_warn, uint32_t max_warn, uint32_t max_error)
  {
    setMinError(min_error);
    setMinWarn(min_warn);
    setWarnError(max_warn,max_error);
  }

  uint32_t AMStat::getMinError() const
  {
    return min_error_;
  }

  void AMStat::setMinError(uint32_t min_error)
  {
    validate_min_=true;
    min_error_ = min_error;
  }

  uint32_t AMStat::getMinWarn() const
  {
    return min_warn_;
  }

  void AMStat::setMinWarn(uint32_t min_warn)
  {
    validate_min_=true;
    min_warn_ = min_warn;
  }
  
  const std::string& AMStat::getShortName() const
  {
    return short_name_;
  }


  /**indicates if max values are assigned */
  const bool AMStat::isValidatingMax() const 
  {
    return validate_max_;
  }

  /**indicates if min values are assigned */
  const bool AMStat::isValidatingMin() const
  {
    return validate_min_;
  }

  /**indicates if this stat has received a value since constructed.*/
  const bool AMStat::isSampleReceived() const
  {
    return sample_received_;
  }

  /**indicates if no samples received results in an error*/
  const bool AMStat::isSampleRequired() const
  {
    return sample_required_;
  }

  void AMStat::setSampleRequired(bool sample_required)
  {
    sample_required_ = sample_required;
  }

  void AMStat::compoundStatus(LifeCycleStatus& status, const LifeCycleStatus new_status)
  {
    if (new_status == LifeCycleStatus::ERROR)
    {
      status = LifeCycleStatus::ERROR;
    }
    else if (new_status == LifeCycleStatus::WARN && status == LifeCycleStatus::OK)
    {
      status = LifeCycleStatus::WARN;
    }
  }
};// namespace am

