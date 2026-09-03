#pragma once

#include <pcl/PointIndices.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>

#include "log.hpp"
#include "cali.hpp"

namespace RingMeasure
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorizeClusters(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        const std::vector<pcl::PointIndices>& cluster_indices);

    template <typename PointT>
    void writeSegmentation (
        const typename pcl::PointCloud<PointT>::Ptr& input_cloud,
        const std::vector<pcl::PointIndices>& indices,
        std::filesystem::path path,
        std::string prefix,
        int num_threshold,
        const std::shared_ptr<spdlog::logger> &logger)
    {
    //   auto logger = spdlog::get("Cali");
      if(!std::filesystem::exists(path))
      {
        bool flag = std::filesystem::create_directories(path);
        if(!flag)
        {
          SPDLOG_LOGGER_ERROR(logger, "Failed to create directory: {}", path.string());
          exit(EXIT_FAILURE);
        }
      }
      typename pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud(input_cloud);
      size_t count = std::min(indices.size(), static_cast<size_t>(20)); // Limit to first 100 clusters
      for (size_t i = 0; i < count; ++i)
      {
          // Create a new cloud object for the current cluster
          typename pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>);
          
          // Create an IndexCollection pointer from the current cluster indices
          pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices(indices[i]));
          
          // Set indices and extract the points
          extract.setIndices(indices_ptr);
          extract.setNegative(false); // 'false' means keep ONLY the specified indices
          extract.filter(*cluster_cloud);
    
          if (cluster_cloud->size() > static_cast<size_t>(num_threshold))
          {   
              SPDLOG_LOGGER_DEBUG(logger, "Cluster {:3d} has {:6d} points.", i, cluster_cloud->size());
              // Save individual cluster to disk
              std::string filename = (path / (prefix + std::to_string(i) + ".ply"));
              SPDLOG_EXEC(logger, spdlog::level::info, pcl::io::savePLYFileASCII(filename, *cluster_cloud));
          }
      }
    
    }

    void visualizeCurvature(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                            float threshold,
                            std::filesystem::path ply_path,
                            const std::shared_ptr<spdlog::logger> &logger);
                          
    void visualizeNormal(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                            std::filesystem::path ply_path,
                            const std::shared_ptr<spdlog::logger> &logger);

    void
    visualizeObliquity(const RingCalibration& cali, fs::path ply_path);
}