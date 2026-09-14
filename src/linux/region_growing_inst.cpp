// region_growing_inst.cpp
#define PCL_NO_PRECOMPILE

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>

// Force the compiler to generate symbols for this exact combination
template class pcl::RegionGrowing<pcl::PointNormal, pcl::PointNormal>;