#ifndef AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_CNT_H
#define AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_CNT_H

#include <cstdint>
#include <limits>

#include <rclcpp/rclcpp.hpp>

//#include <am_utils/am_ros2_utility.h>
#include <super_lib/am_life_cycle_types.h>
#include <diagnostic_updater/diagnostic_updater.hpp>

namespace am
{
/**
 * Additive statistic looking for max thresholds exceeded during the entire Life Cycle.
 * 
 * This is the base statistic that looks for max thresholds for warnings and errors compared to the value. 
 * The value is set or incremented using standard operators (=, +=, ++).
 *
 * See others:
 * 
 * - AMStatReset has min/max thresholds for frequency checking. 
 * - AMStatAve is the average of value over count.   
 * 
 */
class AMStat
{
protected:
  std::string short_name_ = "short";
  std::string long_name_ = "long";
  uint32_t value_ = 0;
  uint32_t max_warn_ = std::numeric_limits<uint32_t>::max();
  uint32_t max_error_ = std::numeric_limits<uint32_t>::max();
  uint32_t min_warn_ = 0;
  uint32_t min_error_ = 0;
  /**indicates if min values are assigned */
  bool validate_min_ = false;
  bool validate_max_ = false;
  bool sample_received_ = false;
  bool sample_required_ = false;
private:
  AMStat();
public:
  AMStat(const std::string& short_name, const std::string& long_name);
  AMStat( const std::string& short_name, const std::string& long_name, uint32_t max_warn, uint32_t max_error);

  AMStat(const std::string& short_name, const std::string& long_name, uint32_t min_error, uint32_t min_warn,
              uint32_t max_warn, uint32_t max_error);

  virtual ~AMStat();

  virtual LifeCycleStatus process(double warn_throttle_s, double error_throttle_s);

  virtual void reset();

  virtual void addStatistics(diagnostic_updater::DiagnosticStatusWrapper& dsw);

  virtual std::string getStrShort();

  virtual std::string getStr();

  virtual void add(uint32_t adder);

  AMStat& operator++(int);

  AMStat& operator+=(int adder);

  AMStat& operator=(uint32_t assignment);

  /** Confusing name for return the value.  Use getValue() instead*/
  [[deprecated]]
  uint32_t getCount() const;

  /**The current value for the stat */
  uint32_t getValue() const;

  const std::string& getLongName() const;

  void setWarnError(uint32_t max_warn, uint32_t max_error);

  uint32_t getMaxError() const;

  void setMaxError(uint32_t max_error);

  uint32_t getMaxWarn() const;

  void setMaxWarn(uint32_t max_warn);

  void setWarnError(uint32_t min_error, uint32_t min_warn, uint32_t max_warn, uint32_t max_error);

  uint32_t getMinError() const;

  void setMinError(uint32_t min_error);

  uint32_t getMinWarn() const;

  void setMinWarn(uint32_t min_warn);
  
  const std::string& getShortName() const;


  /**indicates if max values are assigned */
  const bool isValidatingMax() const;

  /**indicates if min values are assigned */
  const bool isValidatingMin() const;

  /**indicates if this stat has received a value since constructed.*/
  const bool isSampleReceived() const;

  /**indicates if no samples received results in an error*/
  const bool isSampleRequired() const;

  void setSampleRequired(bool sample_required);

  static void compoundStatus(LifeCycleStatus& status, const LifeCycleStatus new_status);
};

};  // namespace am

#endif  // AM_OUSTER_OUSTER_STATS_H
