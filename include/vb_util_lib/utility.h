/*
 * utilit.h
 *
 *  Created on: Apr 9, 2018
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_UTILITY_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_UTILITY_H_

#include <vector>
#include <stdio.h>

#define RED     "\x1b[31m"
#define GREEN   "\x1b[32m"
#define YELLOW  "\x1b[33m"
#define BLUE    "\x1b[34m"
#define MAGENTA "\x1b[35m"
#define CYAN    "\x1b[36m"
#define BRED    "\x1b[31;1m"
#define BGREEN   "\x1b[32;1m"
#define BYELLOW  "\x1b[33;1m"
#define BBLUE    "\x1b[34;1m"
#define BMAGENTA "\x1b[35;1m"
#define BCYAN    "\x1b[36;1m"
#define COLOR_RESET   "\x1b[0m"

namespace am
{
  template <class T>
  inline bool contains(std::vector<T> &src, T key)
  {
    bool result = false;
    for(auto t : src)
    {
      if(t == key)
      {
        result = true;
      }
    }
    return result;
  }
  template<class T>
  inline void mix(std::vector<T> &pot1, std::vector<T> &pot2)
  {
    for(auto m : pot2)
    {
      if(!contains(pot1, m))
      {
        pot1.push_back(m);
      }
    }
  }
  template<class T>
  inline bool getUnion(std::vector<T> &pot1, std::vector<T> &pot2)
  {
    bool result = false;
    for(auto m : pot2)
    {
      if(!contains(pot1, m))
      {
        pot1.push_back(m);
        result = true;
      }
    }
    return result;
  }
  template<class T>
  inline bool areTheyMixable(std::vector<T> &pot1, std::vector<T> &pot2)
  {
    for(auto m : pot1)
    {
      if(contains(pot2, m))
      {
        return true;
      }
    }
    return false;
  }

  template <class T>
  inline std::vector<T> getSubVector(std::vector<T> &src, int start_index, int end_index)
  {
    std::vector<T> result;
    for(int i = start_index; i <= end_index; i++)
    {
      result.push_back(src[i]);
    }
    return result;
  }
  template<class T>
  T abs(T t)
  {
    if(t < 0) return -1 * t;
    else return t;
  }
}



#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_UTILITY_H_ */
