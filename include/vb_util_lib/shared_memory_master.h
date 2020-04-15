/*
 * shared_pointer.h
 *
 *  Created on: Aug 1, 2018
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_SHARED_MEMORY_MASTER_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_SHARED_MEMORY_MASTER_H_

#include <boost/interprocess/shared_memory_object.hpp> //shared memory header
#include <boost/interprocess/managed_shared_memory.hpp> //for  boost::interprocess::managed_shared_memory::handle_t
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud.h>
#include <utility>

namespace am
{
  template<class T>
  class SharedMemoryMaster : public T
  {
  private:
    std::string shared_memory_id_ = "";
    std::string segment_id_ = "segment";
    int data_size_ = 0;
    int data_type_ = -1;
    boost::interprocess::managed_shared_memory *segment = nullptr;
    T *data_ = nullptr;
  public:
    SharedMemoryMaster()
    {

    }
    SharedMemoryMaster(const std::string &shared_memeroy_id, int data_size)
    {
      shared_memory_id_ = shared_memeroy_id;
      data_size_ = data_size;
      //make sure there are no segment previously created
      boost::interprocess::shared_memory_object::remove(shared_memory_id_.c_str());

      segment = new boost::interprocess::managed_shared_memory(boost::interprocess::open_or_create,shared_memory_id_.c_str(), data_size_);

      data_ = segment->construct<T>(segment_id_.c_str());
    }
    ~SharedMemoryMaster()
    {
      delete segment;
      delete data_;
    }

    void setup(const std::string &shared_memeroy_id, int data_size)
    {
      shared_memory_id_ = shared_memeroy_id;
      data_size_ = data_size;
      //make sure there are no segment previously created
      boost::interprocess::shared_memory_object::remove(shared_memory_id_.c_str());

      segment = new boost::interprocess::managed_shared_memory(boost::interprocess::open_or_create,shared_memory_id_.c_str(), data_size_);

      data_ = segment->construct<T>(segment_id_.c_str());
    }
    void write(T &t)
    {
      *data_ = t;
    }
    void read(T &t)
    {
      std::pair<T*, std::size_t> p = segment->find<T>(segment_id_);
      t = *p.first;
    }

  };
}


#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_SHARED_MEMORY_MASTER_H_ */
