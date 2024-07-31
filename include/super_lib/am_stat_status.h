#ifndef AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_STATUS_H
#define AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_STATUS_H

#include <cstdint>
#include <limits>

#include <rclcpp/rclcpp.hpp>

#include <super_lib/am_life_cycle_types.h>
#include <super_lib/am_stat.h>
#include <diagnostic_updater/diagnostic_updater.hpp>

namespace am
{

class AMStatStatus : public AMStat
{
private:
  AMStatStatus();

public:
  AMStatStatus(const std::string& short_name, const std::string& long_name);

  LifeCycleStatus process(double warn_throttle_s, double error_throttle_s) override;

    AMStatStatus& operator=(uint32_t assignment);
};

};  // namespace am

#endif  // AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_STATUS_H
