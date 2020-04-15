/*
 * shared_pointer_client.h
 *
 *  Created on: Aug 7, 2018
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_SHARED_MEMORY_CLIENT_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_SHARED_MEMORY_CLIENT_H_

#include <string>
#include <boost/interprocess/shared_memory_object.hpp>

namespace am
{
template <class T>
class SharedMemoryClient
{
private:
  std::string shared_memory_id_ = "";
  std::string segment_id_ = "segment";
  int data_size_ = 0;
  int data_type_ = -1;
  boost::interprocess::managed_shared_memory *segment = nullptr;
  T *data_ = nullptr;
public:
  SharedMemoryClient()
  {}
  SharedMemoryClient(const std::string &shared_memeroy_id, int data_size)
  {
    shared_memory_id_ = shared_memeroy_id;
    data_size_ = data_size;
    segment = new boost::interprocess::managed_shared_memory(boost::interprocess::open_only, shared_memory_id_.c_str(), data_size_);
    data_ = segment->construct<T>(segment_id_.c_str());
  }
  ~SharedMemoryClient()
  {}
  void setup(const std::string &shared_memeroy_id, int data_size)
  {
    shared_memory_id_ = shared_memeroy_id;
    data_size_ = data_size;
    segment = new boost::interprocess::managed_shared_memory(boost::interprocess::open_or_create,shared_memory_id_.c_str(), data_size_);

    data_ = segment->construct<T>(segment_id_.c_str());
  }
  void read(T &t)
  {
    std::pair<T*, std::size_t> p = segment->find<T>(segment_id_);
    t = *p.first;
  }
};
}




#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_SHARED_MEMORY_CLIENT_H_ */
