#ifndef AM_ONLINESTATS_H
#define AM_ONLINESTATS_H
#include <boost/circular_buffer.hpp>
#include <algorithm>
#include <numeric>
#include <math.h>       /* pow */
#include <vector>
#include <list>
#include <brain_box_msgs/msg/delay_stats.hpp>

namespace am {

class OnlineStatistics {
public:
 OnlineStatistics() : OnlineStatistics(0) {};
 OnlineStatistics(size_t N) : ary_(0), cb_(N) {};
  void addData( double val );
  void setSize( size_t N );
  void insertion_sort(std::vector<double>& ary);
  void setDelayStats( brain_box_msgs::msg::DelayStats &msg );
  inline double getMean() const {return mean_;}
  inline double getVariance() const {return var_;}
  inline double getMin() const {return ary_.front();}
  inline double getMax() const {return ary_.back();}
  inline double getMedian() const {
    return ( ary_.size() % 2 == 0 ? ((ary_[ary_.size()/2-1]+ary_[ary_.size()/2])/2.0) : ary_[ary_.size()/2] );
  }
  friend std::ostream& operator<<(std::ostream& os, OnlineStatistics &stats);
  
private:
  size_t N {0};
  std::vector<double> ary_;
  boost::circular_buffer<double> cb_;
  double mean_ {0.0}; //  mean, median
  double m2n_ {0.0};        // sum of squares of differences from current mean
  double var_ {0.0}, skew_ {0.0}, kurt_ {0.0}; //  variance, skewness, and kurtosis
};


}


#endif
