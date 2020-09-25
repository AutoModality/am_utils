#ifndef AM_UTILS_INCLUDE_AM_UTILS_POINT_TYPE_H_
#define AM_UTILS_INCLUDE_AM_UTILS_POINT_TYPE_H_

#define PCL_NO_PRECOMPILE
#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_types.h>
namespace pcl {
struct PointXYZIR
{
 PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
 float intensity;
 float range;
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

}

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZIR,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (float, range, range)
 )

#endif /*AM_UTILS_INCLUDE_AM_UTILS_POINT_TYPE_H_*/
