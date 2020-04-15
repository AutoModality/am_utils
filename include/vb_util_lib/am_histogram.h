/*
 * am_histogram.h
 *
 *  Created on: May 18, 2016
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_AM_HISTOGRAM_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_AM_HISTOGRAM_H_

#include <ros/ros.h>
//#include <boost/bind.hpp>
//#include <Eigen/StdVector>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/image_encodings.h>

class AM_Histogram {
public:
    AM_Histogram(int bucket_num_, int min_height, int max_height, unsigned int min_bucket_num);

    virtual ~AM_Histogram();

    void clearHistogram();

    void populateHistogram(uint16_t value, float x);

    void addHistogram(AM_Histogram *histogram);

    void finalizeHistogram(bool filter);

    /**
     * Gets the predominant terrain height from the depth image data
     */
    double getPredominantHeightM();

    double getAverageXM();

    void print(std::string prefix="", double delay = 0.0);

    bool isHistogramValid() const {
        return histogram_valid_;
    }

    void setHistogramValid(bool histogramValid) {
        histogram_valid_ = histogramValid;
    }

private:
    ros::Time last_print_time_;

    /**
     * Historgram stuff for the height calculation
     */
    int bucket_num_ {10};
    int max_height_ {1000};
    int min_height_ {1};
    double bucket_width_ {0};
    bool histogram_valid_ {false};
    unsigned int min_bucket_num_;

    struct HistogramBucket {
        unsigned int idx;
        unsigned int num;
        unsigned int filter_num;
        unsigned int weighted_num;
        double total_x;
    };

    HistogramBucket* histogram_ {NULL};
    HistogramBucket* histogram_sort_[3];

    void allocHistogram();
    void updateHistogram(double val);
    void insertHistogramSort(HistogramBucket* b);
    void sortHistogram();
};


#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_AM_HISTOGRAM_H_ */
