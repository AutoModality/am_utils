#include <boost/circular_buffer.hpp>
#include <algorithm>
#include <numeric>
#include <math.h>       /* pow */
#include <vector>
#include <list>
#include "am_utils/am_onlinestatistics.h"

namespace am {

/*
 * @latexonly
 * 
 *
 * @latexend
 *
 */
void OnlineStatistics::addData( double val ) {
    if ( cb.size() == 0 ) {
      mean_ = val;
      var_  = NAN;
      m2n_  = 0.0;
    } else if ( 1 + cb.size() > cb.capacity() ) {
      double tmp = cb.back();
      // std::cout << *this << std::endl;
      double oldmean = mean_;
      mean_ = mean_ + (val - tmp)/cb.size();
      double delta = mean_ - oldmean;
      double alpha = val - tmp;
      /* Fix this part here */
      m2n_ = m2n_ + (delta*delta*cb.size()) + alpha*(alpha+2*tmp-2*mean_ );
      var_ = m2n_ / cb.size();
      auto const ip = std::find( ary_.begin(), ary_.end(), tmp );
      if ( ip != ary_.end() ) {
        ary_.erase(ip);
      }
      
    } else {
      double oldmean_ = mean_;
      mean_ = (val - mean_)/(cb.size()+1) + mean_;
      m2n_ = m2n_ +  ( val - oldmean_)*(val - mean_ );
      var_ = m2n_ / (cb.size()+1);
    }
    ary_.insert(ary_.begin(),val );
    insertion_sort(ary_);
    cb.push_front(val);
    // std::cout << *this << std::endl;
}

void OnlineStatistics::insertion_sort(std::vector<double>& ary) {
  for (auto it = ary.begin(), end = ary.end(); it != end; ++it) {
        // 1. Search
      auto const insertion_point = std::upper_bound(ary.begin(), it, *it);

        // 2. Insert
      std::rotate(insertion_point, it, it + 1);
    }
}

void OnlineStatistics::setSize( size_t N )
{
  cb.set_capacity(N);
  cb.erase(cb.begin(),cb.end());
  ary_.clear();
}
  
void OnlineStatistics::setDelayStats( am_utils::DelayStats &msg )
{
  msg.count = cb.size();
  msg.mean_delay = getMean();
  msg.variance_delay = getVariance();
  msg.min_delay = getMin(); 
  msg.max_delay = getMax();
  msg.median_delay = getMedian();
}
  
std::ostream& operator<<(std::ostream& os, OnlineStatistics &stats) {
  // os << stats.cb;
  os << "[ ";
  for ( auto it = stats.cb.begin() ; it != stats.cb.end(); ++it  ) { 
    os << *it << ( it + 1 == stats.cb.end() ? "" : ", " );
  }
  os << "]";
  return os;
}

}

 
