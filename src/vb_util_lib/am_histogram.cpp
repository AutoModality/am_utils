/*
 * am_histogram.cpp
 *
 *  Created on: May 18, 2016
 *      Author: ubuntu
 */

#include <vb_util_lib/am_histogram.h>

#define HEIGHT(b)	((b)->idx * bucket_width_ + min_height_)

AM_Histogram::AM_Histogram(int bucket_num, int min_height, int max_height, unsigned int min_bucket_num) {
    bucket_num_ = bucket_num;
    min_height_ = min_height;
    max_height_ = max_height;
    bucket_width_ = (max_height_ - min_height_) / bucket_num_;
    min_bucket_num_ = min_bucket_num / bucket_num_;

    allocHistogram();
    clearHistogram();
    last_print_time_ = ros::Time::now();
}

AM_Histogram::~AM_Histogram() {
    if (histogram_ != NULL)
    {
        delete histogram_;
    }
}

void AM_Histogram::print(std::string prefix, double delay) {
    ros::Duration db =  ros::Time::now() - last_print_time_;
    if (db.toSec() < delay)
    {
        return;
    }
    last_print_time_ = ros::Time::now();
    printf("%s ", prefix.c_str());
    for (int i = 0; i <= bucket_num_; i++) {
        printf("%0.2f:%d:%d:%d;%0.2f ", HEIGHT(&histogram_[i]), histogram_[i].num, histogram_[i].filter_num, histogram_[i].weighted_num,
        		(double)histogram_[i].total_x / (double)histogram_[i].num);
    }

    printf("[%0.2f:%0.2f:%0.2f:%0.2f]\n", HEIGHT(histogram_sort_[0]), HEIGHT(histogram_sort_[1]),
    		HEIGHT(histogram_sort_[2]), getPredominantHeightM());
}


double AM_Histogram::getPredominantHeightM() {
	if(!histogram_valid_ || histogram_sort_[0]->num < min_bucket_num_)
	{
		return -1.0;
	}
    // return histogram_sort_[0]->average / 100.0;
    return HEIGHT(histogram_sort_[0]) / 100.0;
}

double AM_Histogram::getAverageXM() {
	if(histogram_sort_[0]->num != 0)
	{
		return histogram_sort_[0]->total_x / histogram_sort_[0]->num;
	}
	else
	{
		return 0.0;
	}
}

void AM_Histogram::allocHistogram() {
    histogram_ = new HistogramBucket[bucket_num_+1];
    for (int i = 0; i <= bucket_num_; i++)
    {
        histogram_[i].idx = i;
    }
}

void AM_Histogram::clearHistogram() {
    for (int i = 0; i <= bucket_num_; i++)
    {
        histogram_[i].num = 0;
        histogram_[i].filter_num = 0;
        histogram_[i].weighted_num = 0;
        histogram_[i].total_x = 0;
    }
}

void AM_Histogram::populateHistogram(uint16_t value, float x) {
    int idx = (value - min_height_) / bucket_width_;
    histogram_[idx].num++;
    histogram_[idx].total_x += x;
    histogram_[bucket_num_].num++;
    histogram_[bucket_num_].total_x += x;
}

void AM_Histogram::addHistogram(AM_Histogram *histogram) {
    for (int i = 0; i <= bucket_num_; i++)
    {
        histogram_[i].num += histogram->histogram_[i].num;
        histogram_[i].total_x += histogram->histogram_[i].total_x;
    }
}

void AM_Histogram::finalizeHistogram(bool filter) {

	unsigned int window = 200 / bucket_width_; // 2m high window

	// filter out top half of points
	unsigned int total_points = 0;
	for(int i = bucket_num_ - 1; i >= 0; i--)
	{
		histogram_[i].filter_num = histogram_[i].num;
		if(filter)
		{
			total_points += histogram_[i].num;
			if(total_points > histogram_[bucket_num_].num / 2)
			{
				break;
			}
		}
	}

	unsigned int max = 0;
	for(int i = 0; i < bucket_num_; i++)
	{
		int start = i - window;
		start = start < 0 ? 0 : start;
		int end = i + window;
		end = end >= bucket_num_  ? bucket_num_ -1 : end;
		histogram_[i].weighted_num = 0;
		for(int j = start; j < end; j++)
		{
			// appoxiamte gaussian
			int distance = abs(i - j);
			float mutiplier = (float)(window + 1 - distance) / (float)(window + 1);
			histogram_[i].weighted_num += (unsigned int)(histogram_[j].filter_num * mutiplier + 0.5);
		}
		max = histogram_[i].weighted_num > max ? histogram_[i].weighted_num : max;
	}


	sortHistogram();

	histogram_valid_ = true;

}

void AM_Histogram::updateHistogram(double val) {
}

void AM_Histogram::insertHistogramSort(HistogramBucket* b) {
    for (int i = 0; i < 3; i++)
    {
        if (histogram_sort_[i] == NULL)
        {
            histogram_sort_[i] = b;
            return;
        }
        if (b->weighted_num > histogram_sort_[i]->weighted_num)
        {
            HistogramBucket* n = b;
            for (int j = i; j < 3; j++)
            {
                HistogramBucket* o = histogram_sort_[j];
                histogram_sort_[j] = n;
                n = o;
            }
            return;
        }
    }
}

void AM_Histogram::sortHistogram() {
    for (int i = 0; i < 3; i++)
    {
        histogram_sort_[i] = NULL;
    }

    for (int i = 0; i < bucket_num_; i++)
    {
        insertHistogramSort(histogram_ + i);
    }
}


