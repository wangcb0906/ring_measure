#pragma once

#include <pcl/PointIndices.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>

#include "log.hpp"
#include "cali.hpp"

namespace RingMeasure
{
#if SPDLOG_LEVEL_DEBUG >= ACTIVE_LOG_LEVEL
    /**
     * @brief Colorizes segmented point cloud clusters with RGB values.
     * The colors are randomly assigned to each cluster for visualization purposes.
     * 
     * @param input_cloud Pointer to the original input point cloud (PointXYZ).
     * @param cluster_indices Vector containing index sets for each extracted cluster.
     * @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr A new point cloud with color-coded clusters.
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorizeClusters(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        const std::vector<pcl::PointIndices>& cluster_indices);

    /**
     * @brief Writes individual point cloud clusters to disk as PLY files.
     * 
     * @tparam PointT The point type (e.g., pcl::PointXYZ, pcl::PointNormal).
     * @param input_cloud Pointer to the source point cloud.
     * @param indices Vector of point indices defining the clusters.
     * @param path Target directory path where cluster files will be saved.
     * @param prefix String prefix to prepend to each saved cluster filename.
     * @param num_threshold Minimum point count threshold; clusters smaller than this are ignored.
     * @param logger Shared pointer to the spdlog logger instance.
     */
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
              std::string filename = (path / (prefix + std::to_string(i) + ".ply")).string();
              SPDLOG_EXEC(logger, spdlog::level::info, savePLY(filename, *cluster_cloud));
          }
      }
    
    }

    /**
     * @brief Visualizes and exports curvature data for point clouds.
     * 
     * @param cloud Pointer to the input point cloud with normal data.
     * @param threshold Curvature threshold value used for filtering or highlighting.
     *                  If threshold is not zero, points will be binarized.
     *                  Otherwise the curvature values will be visualized with max-min normalization.
     * @param ply_path File path to export the resulting PLY visualization.
     * @param logger Shared pointer to the spdlog logger instance.
     */
    void visualizeCurvature(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                            float threshold,
                            std::filesystem::path ply_path,
                            const std::shared_ptr<spdlog::logger> &logger);

    /**
     * @brief Visualizes and exports surface normals for point clouds.
     * @details The point cloud are colored based on the angle between the normal vector and the y-axis.
     * 
     * @param cloud Pointer to the input point cloud containing normals.
     * @param ply_path File path to export the resulting PLY visualization.
     * @param logger Shared pointer to the spdlog logger instance.
     */
    void visualizeNormal(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                            std::filesystem::path ply_path,
                            const std::shared_ptr<spdlog::logger> &logger);

    /**
     * @brief Visualizes obliquity characteristics based on ring calibration data.
     * @details This function put the three based together and colorizes the point cloud based
     *          on y coordinates within itself. The consistency of the order of the colors proves
     *          a good calibration.
     * 
     * @param cali Reference to the RingCalibration parameters.
     * @param ply_path File path to save or load the associated point cloud data.
     */
    void
    visualizeObliquity(const RingCalibration& cali, fs::path ply_path);
#endif
}