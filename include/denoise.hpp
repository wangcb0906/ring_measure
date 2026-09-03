#pragma once

#include "config.hpp"
#include <pcl/point_types.h>
#include <functional>
#include <filesystem>

namespace RingMeasure
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr ecExtract(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        float& cluster_tolerance,
        int min_cluster_size,
        std::filesystem::path path,
        std::string prefix,
        const RingMeasure::ScanLocation& scan_loc,
        const std::shared_ptr<spdlog::logger> &logger);
    pcl::PointCloud<pcl::PointNormal>::Ptr normalSegmentation(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        int& camera_resolution,
        const float& mid_height,
        std::filesystem::path path,
        std::string prefix,
        pcl::PointCloud<pcl::PointNormal>::Ptr& lateral,
        pcl::PointCloud<pcl::PointNormal>::Ptr& base,
        const RingMeasure::ScanLocation& scan_loc,
        const std::shared_ptr<spdlog::logger> &logger);
}