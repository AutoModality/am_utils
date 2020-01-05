#ifndef AM_ONLINESTATS_H
#define AM_ONLINESTATS_H
#include <boost/circular_buffer.hpp>
#include <algorithm>
#include <numeric>
#include <math.h>       /* pow */
#include <vector>
#include <list>
#include "am_utils/DelayStats.h"

namespace am {

class OnlineStatistics {
public:
 OnlineStatistics() : OnlineStatistics(0) {};
 OnlineStatistics(size_t N) : ary_(0), cb_(N) {};
  void addData( double val );
  void setSize( size_t N );
  void insertion_sort(std::vector<double>& ary);
  void setDelayStats( am_utils::DelayStats &msg );
  inline double getMean() const {return mean_;}
  inline double getVariance() const {return var_;}
  inline double getMin() const {return ary_.front();}
  inline double getMax() const {return ary_.back();}
  inline double getMedian() const {
    return ( ary_.size() % 2 == 0 ? ((ary_[ary_.size()/2-1]+ary_[ary_.size()/2])/2.0) : ary_[ary_.size()/2] );
  }
  friend std::ostream& operator<<(std::ostream& os, OnlineStatistics &stats);
  
private:
  size_t N;
  std::vector<double> ary_;
  boost::circular_buffer<double> cb_;
  double mean_; //  mean, median
  double m2n_;        // sum of squares of differences from current mean
  double var_, skew_, kurt_; //  variance, skewness, and kurtosis
};


}


#endif
