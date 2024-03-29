#ifndef AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_AVE_H
#define AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_AVE_H

#include <super_lib/am_stat_reset.h>

namespace am
{

/** Calculates the recent average, min and max of a value, over the past second since it
 * is reset upon the Life Cycle heartbeat.
 */
class AMStatAve : public AMStatReset
{
protected:
  uint64_t total_ = 0;
  uint32_t max_ = 0;
  uint32_t min_ = std::numeric_limits<uint32_t>::max();
  uint32_t min_min_warn_ = 0;
  uint32_t min_min_error_ = 0;
  uint32_t max_max_warn_ = std::numeric_limits<uint32_t>::max();
  uint32_t max_max_error_ = std::numeric_limits<uint32_t>::max();

private:
  AMStatAve();

public:
  AMStatAve(const std::string& short_name, const std::string& long_name);

  AMStatAve(const std::string& short_name, const std::string& long_name, uint32_t max_warn, uint32_t max_error);

  AMStatAve(const std::string& short_name, const std::string& long_name, uint32_t min_error, uint32_t min_warn,
            uint32_t max_warn, uint32_t max_error);
  AMStatAve(const std::string& short_name, const std::string& long_name, uint32_t min_error, uint32_t min_warn,
            uint32_t max_warn, uint32_t max_error, uint32_t min_min_error = 0, uint32_t min_min_warn = 0,
            uint32_t max_max_warn = std::numeric_limits<uint32_t>::max(),
            uint32_t max_max_error = std::numeric_limits<uint32_t>::max());
  LifeCycleStatus process(double warn_throttle_s, double error_throttle_s) override;
  void reset() override;
  void add(uint32_t value) override;
  uint32_t getAve();
  void addStatistics(diagnostic_updater::DiagnosticStatusWrapper& dsw) override;
  std::string getStrShort() override;
  std::string getStr() override;
  uint32_t getMax() const;
  uint32_t getMaxMaxError() const;
  uint32_t getMaxMaxWarn() const;
  uint32_t getMin() const;
  uint32_t getMinMinError() const;
  uint32_t getMinMinWarn() const;
  uint64_t getTotal() const;
  void setMax(uint32_t max = 0);
  void setMaxMaxError(uint32_t maxMaxError = std::numeric_limits<uint32_t>::max());

  void setMaxMaxWarn(uint32_t maxMaxWarn = std::numeric_limits<uint32_t>::max());

  void setMin(uint32_t min = std::numeric_limits<uint32_t>::max());

  void setMinMinError(uint32_t minMinError = 0);

  void setMinMinWarn(uint32_t minMinWarn = 0);
};

};  // namespace am

#endif  // AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_AVE_H
