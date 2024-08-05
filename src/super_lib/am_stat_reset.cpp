#include <am_utils/am_ros2_utility.h>
#include <super_lib/am_stat_reset.h>

namespace am
{
  AMStatReset::AMStatReset(const std::string& short_name, const std::string& long_name) : AMStat(short_name, long_name)
  {
    init();
  }

  AMStatReset::AMStatReset(const std::string& short_name, const std::string& long_name, uint32_t max_warn, uint32_t max_error)
    : AMStat(short_name, long_name, max_warn, max_error)
    {
      init();
    }

  AMStatReset::AMStatReset(const std::string& short_name, const std::string& long_name, uint32_t min_error, uint32_t min_warn,
              uint32_t max_warn, uint32_t max_error)
    : AMStat(short_name, long_name,min_error,min_warn, max_warn, max_error)
    {
      init();
    }

  void AMStatReset::init()
  {
    sample_required_ = true;
  }

  void AMStatReset::reset()
  {
    value_ = 0;
  }
  
  AMStatReset& AMStatReset::operator=(uint32_t assignment)
  {
    sample_received_ = true;
    value_ = assignment;
    return *this;
  }


};// namespace am

