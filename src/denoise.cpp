
#include <pcl/io/ply_io.h>
#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ml/kmeans.h>

#include <chrono>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/std.h>
#include <fmt/ranges.h>
// k means algorithm
#include <opencv2/core.hpp>

#include "visualization.hpp"

namespace {
    namespace spdl=spdlog::level;
    struct RGBColor {
        uint8_t r, g, b;
    };
    // Classic Distinct Palette (Red, Green, Blue, Yellow)
    std::vector<RGBColor> cluster_colors = {
        {255, 0,   0},   // Cluster 0: Pure Red
        {0,   255, 0},   // Cluster 1: Pure Green
        {0,   0,   255}, // Cluster 2: Pure Blue
        {255, 255, 0}    // Cluster 3: Yellow
    };
    pcl::PointCloud<pcl::PointNormal>::Ptr
    removeCurvatureNan(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
                       const std::shared_ptr<spdlog::logger> &logger)
    {
        size_t nan_curvature_count = 0;
        pcl::PointCloud<pcl::PointNormal>::Ptr valid_points(new pcl::PointCloud<pcl::PointNormal>);
        valid_points->reserve(cloud->size());
        for (const auto& pt : *cloud) {
            if (!std::isnan(pt.curvature) && !std::isinf(pt.curvature)) {
                valid_points->push_back(pt);
            } else {
                SPDLOG_LOGGER_TRACE(logger, "Skipping point with NaN or Inf curvature: ({}, {}, {})", pt.x, pt.y, pt.z);
                nan_curvature_count++;
            }
        }
        SPDLOG_LOGGER_DEBUG(logger, "Number of points with NaN curvature: {}/{}", nan_curvature_count, cloud->size());
        return valid_points;
    }
    
    // The curvature in PCL is stored as float.
    float getCurvThld(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
                      const std::filesystem::path ply_path,
                      const RingMeasure::RegionGrowing& params,
                      const std::shared_ptr<spdlog::logger> &logger)
    {

        /// Mode1 is driven by kmeans clustering.
        /// Mode2 is driven by the percentile setting, which depends on an extra parameter.
        /// Mode3 takes the smaller value from MODE1 and MODE2, which is more robust to noise
        /// at the risk of discarding some valid points.
        auto mode = params.curv_thld_mode;

        std::vector<std::vector<float>> points;
        points.reserve(cloud->size());
        for (const auto& pt : *cloud) {
            points.push_back({pt.curvature});
        }
        
        float kmeans_curv_thld;
        if(mode != RingMeasure::Modes::MODE2){
            pcl::Kmeans kmeans(cloud->size(), 1u);
            kmeans.setClusterSize(2);
            kmeans.setInputData(points);
            kmeans.kMeans();
            auto centroids = kmeans.get_centroids();
            std::vector<size_t> cluster_sizes(centroids.size(), 0);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            SPDLOG_EXEC(logger, spdl::trace, colored_cloud->resize(cloud->size()););
            for (size_t i = 0; i < cloud->size(); ++i) {
                const auto& pt = cloud->points[i];
                float curv = pt.curvature;
                int best_cluster = 0;
                float min_dist = std::numeric_limits<float>::max();
                for (size_t c = 0; c < centroids.size(); ++c) {
                    float dist = std::abs(curv - centroids[c][0]);
                    if (dist < min_dist) {
                        min_dist = dist;
                        best_cluster = static_cast<int>(c);
                    }
                }
                SPDLOG_EXEC(logger, spdl::trace,
                    auto& dest = colored_cloud->points[i];
                    dest.x = pt.x;
                    dest.y = pt.y;
                    dest.z = pt.z;
                    dest.r = cluster_colors[best_cluster].r;
                    dest.g = cluster_colors[best_cluster].g;
                    dest.b = cluster_colors[best_cluster].b;
                );
                cluster_sizes[best_cluster]++;
            }
            SPDLOG_LOGGER_TRACE(logger, "Cluster sizes: {}", fmt::join(cluster_sizes, ", "));
            SPDLOG_LOGGER_TRACE(logger, "Saving the visualized clustering of normal to {}.", ply_path);
            SPDLOG_EXEC(logger, spdl::trace, pcl::io::savePLYFileASCII(ply_path, *colored_cloud));
            kmeans_curv_thld = (centroids[0][0] + centroids[1][0]) / 2.0f;
            SPDLOG_LOGGER_TRACE(logger, "Curvature threshold determined by k-means: {}", kmeans_curv_thld);
        }

        float percentile_curv_thld;
        if(mode != RingMeasure::Modes::MODE1){
            std::sort(points.begin(), points.end(), 
                [](const std::vector<float>& a, const std::vector<float>& b) {
                  return a[0] > b[0]; // Greater than for descending order
                }
            );
            float ratio = params.curv_thld_ratio;
            int percentile_index = 360 * ratio;
            percentile_curv_thld = points[percentile_index][0];
            SPDLOG_LOGGER_TRACE(logger, "Percentile index for curvature threshold: {} and value: {}", percentile_index, percentile_curv_thld);
        }
        float curv_thld = 0.0f;
        switch(mode){
            case RingMeasure::Modes::MODE1:
                curv_thld = kmeans_curv_thld;
                break;
            case RingMeasure::Modes::MODE2:
                curv_thld = percentile_curv_thld;
                break;
            case RingMeasure::Modes::MODE3:
                curv_thld = std::min(kmeans_curv_thld, percentile_curv_thld);
                break;
            default:
                SPDLOG_LOGGER_ERROR(logger, "Unknown curvature threshold mode: {}", static_cast<int>(mode));
                throw std::runtime_error("Unknown curvature threshold mode");
        }

        SPDLOG_LOGGER_DEBUG(logger, "Final curvature threshold: {}", curv_thld);
        return curv_thld;
    }

    // The normal in PCL is stored as float.
    float getNormalThld(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
                        const std::filesystem::path ply_path,
                        const RingMeasure::RegionGrowing& params,
                        const std::shared_ptr<spdlog::logger> &logger)
    {

        pcl::Kmeans kmeans(cloud->size(), 1u);
        kmeans.setClusterSize(4);
        for (const auto& pt : *cloud) {
            std::vector<float> vec;
            vec.push_back(std::abs(pt.normal_y));
            kmeans.addDataPoint(vec);
        }
        kmeans.kMeans();
        auto centroids_ = kmeans.get_centroids();
        std::vector<float> centroids(centroids_.size());
        std::transform(centroids_.begin(), centroids_.end(), centroids.begin(), [](const std::vector<float>& row) {
            return row[0];
        });

        SPDLOG_LOGGER_DEBUG(logger, "K-means centroids for normal_y: {}", fmt::join(centroids, ", "));
        
        std::vector<size_t> cluster_sizes(centroids.size(), 0);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        SPDLOG_EXEC(logger, spdl::trace, colored_cloud->resize(cloud->size()););
        for (size_t i = 0; i < cloud->size(); ++i) {
            const auto& pt = cloud->points[i];
            float normal_y = std::abs(pt.normal_y);
            int best_cluster = 0;
            float min_dist = std::numeric_limits<float>::max();
            for (size_t c = 0; c < centroids.size(); ++c) {
                // Calculate Euclidean distance to centroid c
                float dist = std::abs(normal_y - centroids[c]);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_cluster = static_cast<int>(c);
                }
            }
            cluster_sizes[best_cluster]++;
            SPDLOG_EXEC(logger, spdl::trace,
                auto& dest = colored_cloud->points[i];
                dest.x = pt.x;
                dest.y = pt.y;
                dest.z = pt.z;
                dest.r = cluster_colors[best_cluster].r;
                dest.g = cluster_colors[best_cluster].g;
                dest.b = cluster_colors[best_cluster].b;
            );
        }
        SPDLOG_LOGGER_DEBUG(logger, "K-means cluster sizes: {}", fmt::join(cluster_sizes, ", "));
        centroids.erase(
            std::remove_if(centroids.begin(), centroids.end(), [](const float& val) {
                return std::isnan(val);
            }),
            centroids.end()
        );
        SPDLOG_LOGGER_TRACE(logger, "Before sorting: centroids = [{}]", fmt::join(centroids, ", "));
        // Sort the array in descending order becasue acos is a decreasing function.
        std::sort(centroids.begin(), centroids.end(), [](const float& a, const float& b) {
            return a > b;
        });
        SPDLOG_LOGGER_TRACE(logger, "After filtering and sorting: centroids = [{}]", fmt::join(centroids, ", "));
        std::for_each(centroids.begin(), centroids.end(), [](float& val) {
            val = std::acos(val);
        });
        SPDLOG_LOGGER_TRACE(logger, "Convert to radians: centroids = [{}]", fmt::join(centroids, ", "));
        std::vector<float> diffs(centroids.size());
        std::adjacent_difference(centroids.begin(), centroids.end(), diffs.begin());
        // skip the first element of diffs since it's not a valid difference
        // 2 points divided by 3
        // 3 points divded by 2
        // 4 points, keep the minimal
        float thld = *std::min_element(diffs.begin() + 1, diffs.end()) / (5 - diffs.size());
        SPDLOG_LOGGER_TRACE(logger, "Size of differences: {}", diffs.size());
        SPDLOG_LOGGER_TRACE(logger, "Differences between adjacent centroids: {}", fmt::join(diffs, ", "));
        SPDLOG_LOGGER_TRACE(logger, "Saving the visualized clustering of normal to {}.", ply_path);
        SPDLOG_EXEC(logger, spdl::trace, pcl::io::savePLYFileASCII(ply_path, *colored_cloud));
        // Take the average of the first two centroids as the normal threshold
        SPDLOG_LOGGER_DEBUG(logger, "Normal threshold determined by k-means: {} degrees", thld * 180.0f / M_PI);
        return thld;
    }
}

namespace RingMeasure {
    namespace rm = ::RingMeasure;
    pcl::PointCloud<pcl::PointXYZ>::Ptr RingCalibration::ecExtract(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        int min_cluster_size,
        fs::path path,
        const std::string prefix)
    {
        // namespace spdl=spdlog::level;
        // get the logger system
        // auto logger = spdlog::get("Cali");
        // 1. Create a KdTree for fast spatial search
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(input);
      
        // 2. Vector to hold indices of each detected cluster
        std::vector<pcl::PointIndices> ec_out_indices;
      
        // 3. Configure EuclideanClusterExtractor
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(camera_resolution + 1.5f);
        ec.setMinClusterSize(min_cluster_size);
        ec.setMaxClusterSize(input->size());
        ec.setSearchMethod(tree);
        ec.setInputCloud(input);
      
        // 4. Extract clusters
        {
            auto start_time = std::chrono::high_resolution_clock::now();
            ec.extract(ec_out_indices);
            auto end_time = std::chrono::high_resolution_clock::now();
            SPDLOG_LOGGER_INFO(logger, "Euclidean Cluster Extraction completed in {} ms", 
                std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count());
        }
      
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_ec_out = rm::colorizeClusters(input, ec_out_indices);
      
        std::sort(ec_out_indices.begin(), ec_out_indices.end(), [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
            return a.indices.size() > b.indices.size(); // Sort in descending order of cluster size
        });
      
        SPDLOG_LOGGER_DEBUG(logger, "Writing the {} clusters of out denoised to: {}", ec_out_indices.size(), path);
        SPDLOG_EXEC(logger, spdl::debug, rm::writeSegmentation<pcl::PointXYZ>(input, ec_out_indices, path, prefix + "ECSeg_", 100, logger));
        // rm::writeSegmentation<pcl::PointXYZ>(input, ec_out_indices, path, prefix + "ECSeg_", 100, logger);
        // write the point cloud to disk 
        SPDLOG_LOGGER_INFO(logger, "Saving denoised reconstruction to {}", path / (prefix + "EC_out.ply"));
        SPDLOG_EXEC(logger, spdl::info, pcl::io::savePLYFileASCII(path / (prefix + "EC_out.ply"), *colored_ec_out));
      
        /*********************************************************************************************** */
        // The GPU Euclidean Cluster Extraction is slower than the CPU version.
        // Instead of using the CPU version, we downsample the point cloud.
        // pcl::gpu::Octree::PointCloud cloud_device;
        // cloud_device.upload(out->points);
        
        // pcl::gpu::Octree::Ptr octree_device (new pcl::gpu::Octree);
        // octree_device->setCloud(cloud_device);
        // octree_device->build();
      
        // std::vector<pcl::PointIndices> ec_out_indices_gpu;
        // pcl::gpu::EuclideanClusterExtraction<pcl::PointXYZ> gec;
        // gec.setClusterTolerance (cluster_tolerance);
        // gec.setMinClusterSize (100);
        // gec.setMaxClusterSize (out->size());
        // gec.setSearchMethod (octree_device);
        // gec.setHostCloud (out);
        // {
        // auto start_time = std::chrono::high_resolution_clock::now();
        // gec.extract (ec_out_indices_gpu);
        // auto end_time = std::chrono::high_resolution_clock::now();
        // SPDLOG_LOGGER_INFO(logger, "GPU Euclidean Cluster Extraction completed in {} ms", 
        //              std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count());
        // }
      
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_ec_out_gpu = colorizeClusters(out, ec_out_indices_gpu);
        // // write the point cloud to disk 
        // pcl::io::savePLYFileASCII(result / "EC_out_gpu.ply", *colored_ec_out_gpu);
        /************************************************************************************************** */

        // For under, we first apply the clustering algorithm to the point cloud, and then select the four clusters
        // with the largest y coordinate based on the observation.
        pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
        if (scan_loc == rm::ScanLocation::Under)
        {
            int num_of_segs = ec_out_indices.size();
            cv::Mat feature(num_of_segs, 1, CV_32F); // the feature vector is the size of the clusters
            int k = 2; // number of clusters in k-means
            for (int i = 0; i < num_of_segs; ++i)
            {
                feature.at<float>(i, 0) = ec_out_indices[i].indices.size();
            }
            cv::Mat labels, centers;
            cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 1000, 1.0);
            cv::kmeans(feature, k, labels, criteria, 3, cv::KMEANS_PP_CENTERS, centers);
            cv::Point max_loc;
            cv::minMaxLoc(centers, nullptr, nullptr, nullptr, &max_loc);
            int target = max_loc.y; // the cluster with the largest size
            std::vector<pcl::PointIndices> target_clusters;
            for (int i = 0; i < num_of_segs; ++i){
                
                if (labels.at<int>(i, 0) == target)
                {
                    target_clusters.push_back(ec_out_indices[i]);
                }
            }
            // sort the first clusters based on the y coordinate of the centroid
            std::sort(target_clusters.begin(), target_clusters.end(), [&input](const pcl::PointIndices& a, const pcl::PointIndices& b) {
                Eigen::Vector4f centroid_a, centroid_b;
                pcl::compute3DCentroid(*input, a.indices, centroid_a);
                pcl::compute3DCentroid(*input, b.indices, centroid_b);
                return centroid_a.y() > centroid_b.y(); // Sort in descending order of y coordinate
            });
            // take the first four clusters according to the oberservation
            // The parameter four is an observation based on the point cloud data.
            int size = std::min(static_cast<size_t>(4), target_clusters.size());
            for(int i=0; i<size; ++i){
                *out += pcl::PointCloud<pcl::PointXYZ>(*input, target_clusters[i].indices);
            }
        } else 
        /* For out and in, select the PC with highest y coordinate within the first k largest PCs*/
        {
            int k=std::min(static_cast<size_t>(5), ec_out_indices.size());
      
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*input, ec_out_indices[0].indices, centroid);
            int out_denoise_selcted = 0;
            float max_centroid_y = centroid.y(); 
            // Extract the selected denoised cluster and save it to disk.
            // This cluster would also be further processed for the region growing segmentation.
            for (int i=1; i<k; ++i)
            {
                pcl::compute3DCentroid(*input, ec_out_indices[i].indices, centroid);
                // How to configure the functional?
                // This function aims to find the cluster with maximum y coordinate, which is the ring.
                // The y coordinate of other clusters should be smaller than the ring, becasue of the obervation
                // that all the noise are under ring. So we set the functional to be std::less<float>().
                if (!(centroid.y() < max_centroid_y))
                {
                    out_denoise_selcted = i;
                    max_centroid_y = centroid.y();
                }
            }
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(input);
            extract.setIndices(pcl::make_shared<pcl::PointIndices>(std::move(ec_out_indices[out_denoise_selcted])));
            extract.setNegative(false); // Keep only the specified indices
            extract.filter(*out);
        }
        return out;
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr RingCalibration::normalSegmentation(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        std::filesystem::path path,
        std::string prefix,
        pcl::PointCloud<pcl::PointNormal>::Ptr& lateral,
        pcl::PointCloud<pcl::PointNormal>::Ptr& base)
        {
            auto params = getParam().reg_growing;
            // get the logger system
            // auto logger = spdlog::get("Cali");
            namespace spdl=spdl;
            pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
            // set the search method for the normal estimation
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud (input);
            normal_estimator.setSearchMethod (tree);
            normal_estimator.setInputCloud (input);
            normal_estimator.setRadiusSearch (camera_resolution*params.radius_ratio);
            // Set all the normals point into the center of the cylinder, which is (0, mid_height, 0)
            // later we can reverse the normals to point outside of the surface
            // which is a requirement for many applications.
            normal_estimator.setViewPoint (0.0f, mid_height, 0.0f);
            // normal_estimator.setKSearch (500);
            normal_estimator.compute (*normals);
            // Flip every normal to point AWAY from (0,cynlinder_center,0)
            for (auto& n : normals->points) {
                n.normal_x = -n.normal_x;
                n.normal_y = -n.normal_y;
                n.normal_z = -n.normal_z;
            }
            
            pcl::PointCloud<pcl::PointNormal>::Ptr input_with_normals (new pcl::PointCloud<pcl::PointNormal>);
            pcl::concatenateFields(*input, *normals, *input_with_normals);
            
            pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
            reg.setResidualThreshold (camera_resolution * params.residual_ratio);
            reg.setMinClusterSize (params.min_cluster_size);
            reg.setMaxClusterSize (input->size());
            tree->setInputCloud (input);
            reg.setSearchMethod (tree);
            reg.setNumberOfNeighbours (params.num_neighbors);
            reg.setInputCloud (input);
            reg.setInputNormals (normals);

            auto nan_curv_removed =  removeCurvatureNan(input_with_normals, logger);
            float curv_thld = 0.0;
            if(params.curvature_type == OperationMode::MANUAL)
            {
                reg.setCurvatureThreshold (params.curvature_thld);
            } else if(params.curvature_type == OperationMode::AUTO)
            {
                curv_thld = getCurvThld(nan_curv_removed, path / (prefix + "curv_clustering_visualization.ply"), params, logger);
                reg.setCurvatureThreshold (curv_thld);
                SPDLOG_LOGGER_INFO(logger, "Curvature threshold is automatically determined.");
            } else 
            {
                SPDLOG_LOGGER_ERROR(logger, "Invalid curvature threshold type: {}", static_cast<int>(params.curvature_type));
                exit(EXIT_FAILURE);
            }

            if(params.normal_type == OperationMode::MANUAL)
            {
                reg.setSmoothnessThreshold (params.smooth_thld);
            } else if(params.normal_type == OperationMode::AUTO)
            {
                SPDLOG_LOGGER_INFO(logger, "Normal threshold is automatically determined.");
                float normal_thld = getNormalThld(nan_curv_removed, path / (prefix + "normal_clustering_visualization.ply"), params,logger);
                reg.setSmoothnessThreshold (normal_thld);
                SPDLOG_LOGGER_INFO(logger, "Smoothness threshold is automatically determined.");
            } else 
            {
                SPDLOG_LOGGER_ERROR(logger, "Invalid smoothness threshold type: {}", static_cast<int>(params.normal_type));
                exit(EXIT_FAILURE);
            }

            SPDLOG_EXEC(logger, spdl::debug, visualizeCurvature(nan_curv_removed, curv_thld, 
                path / (prefix + "curvature_binary.ply"), logger));
            SPDLOG_EXEC(logger, spdl::debug, visualizeCurvature(nan_curv_removed, 0.0, 
                path / (prefix + "curvature_max_min.ply"), logger));
            SPDLOG_EXEC(logger, spdl::debug, visualizeNormal(nan_curv_removed,
                path / (prefix + "normal_normalized_visualization.ply"), logger));
            
            std::vector <pcl::PointIndices> segmentation_indices;
            reg.extract (segmentation_indices);

            std::sort(segmentation_indices.begin(), segmentation_indices.end(),
                     [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
                     return a.indices.size() > b.indices.size(); // Sort in descending order of cluster size
            }); 
            
            SPDLOG_LOGGER_DEBUG(logger, "Write the {} clusters of out segmented to: {}", segmentation_indices.size(), path);
            SPDLOG_EXEC(logger, spdl::debug, rm::writeSegmentation<pcl::PointNormal>(input_with_normals, segmentation_indices, path, prefix + "RGSeg_", 10, logger));

            SPDLOG_LOGGER_DEBUG(logger, "Saving the downsampled point cloud ({} points) with normals to {}",
                          input_with_normals->size(), path / (prefix + "normal.ply"));
            SPDLOG_EXEC(logger, spdl::debug, pcl::io::savePLYFileASCII(path / (prefix + "normal.ply"), *input_with_normals));

            pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
            SPDLOG_LOGGER_INFO(logger, "Saving the colored segmented point cloud to {}", path / (prefix + "segmented.ply"));
            SPDLOG_EXEC(logger, spdl::info, pcl::io::savePLYFileASCII(path / (prefix + "segmented.ply"), *colored_cloud));
            if (segmentation_indices.size() < 2)
            {
              SPDLOG_LOGGER_ERROR(logger, "Failed to segment.");
              exit(EXIT_FAILURE);
            }
            
            if (scan_loc != rm::ScanLocation::Under){
                // select the cluster lateral and the base
                int lateral_idx = 0, base_idx = 1;
                Eigen::Vector4f lateral_centroid, base_centroid;
                pcl::compute3DCentroid(*input, segmentation_indices[lateral_idx], lateral_centroid);
                pcl::compute3DCentroid(*input, segmentation_indices[base_idx], base_centroid);
                // How to configure the functional?
                // This function aims to distinguish the base and the lateral of the cylinder. And we want to
                // select the lateral of the cone, which has a smaller y coordinate than the base.
                // So we set the functional to be std::less<float>().
                if (!(lateral_centroid.y() < base_centroid.y()))
                {
                    std::swap(lateral_idx, base_idx);
                }
     
                // Extract the lateral and base clusters 
                pcl::ExtractIndices<pcl::PointNormal> extract;
                extract.setInputCloud(input_with_normals);
     
                if(lateral != nullptr){
                    // lateral
                    pcl::PointIndices::Ptr selected_indices_ptr(new pcl::PointIndices(segmentation_indices[lateral_idx]));
                    extract.setIndices(selected_indices_ptr);
                    extract.setNegative(false); // Keep only the specified indices
                    extract.filter(*lateral);
                }

                if(base != nullptr){
                    // base
                    pcl::PointIndices::Ptr selected_indices_ptr(new pcl::PointIndices(segmentation_indices[base_idx]));
                    extract.setIndices(selected_indices_ptr);
                    extract.setNegative(false); // Keep only the specified indices
                    extract.filter(*base);
                }
            } else 
            // Processing of segmentation for the underneath scan.
            // First cluster the segmengts according to the size of the segmengts,
            // This step is designed to remove the noisy small segments.
            // Then cluster the remaining segments according to the y coordinate
            // of the centroid to separate the base and the lateral.
            {
                auto seg_indices = segmentation_indices;
                int num_of_segs = seg_indices.size();
                cv::Mat feature(num_of_segs, 1, CV_32F); // the feature vector is the size of the clusters
                // The k is set based on the observation of the point cloud data.
                // The scan from under consists of three four parts, which are separated by the cross.
                // So the base and the lateral are separated into more smaller segments.
                // We usually have three kinds of segments, the first kind with the most points,
                // which are parts of the lateral, the second kind with moderate number of points, which are
                // parts of the base, and the third kind with the least number of points, which are noise.
                int k = params.cluster_size; // number of clusters in k-means
                for (int i = 0; i < num_of_segs; ++i)
                {
                    feature.at<float>(i, 0) = seg_indices[i].indices.size();
                }
                cv::Mat labels, centers;
                cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 1000, 1.0);
                cv::kmeans(feature, k, labels, criteria, 3, cv::KMEANS_PP_CENTERS, centers);
                // This cluster is to discard.
                cv::Point min_loc;
                cv::minMaxLoc(centers, nullptr, nullptr, &min_loc, nullptr);
                int dsicard = min_loc.y; // the cluster with the smallest size
                std::vector<pcl::PointIndices> target_clusters;
                for (int i = 0; i < num_of_segs; ++i){
                    
                    if (labels.at<int>(i, 0) != dsicard)
                    {
                        target_clusters.push_back(seg_indices[i]);
                    }
                }

                cv::Mat feature_y(target_clusters.size(), 1, CV_32F); // the feature vector is the y coordinate of the centroid
                for (int i = 0; i < target_clusters.size(); ++i)
                {
                    Eigen::Vector4f centroid;
                    pcl::compute3DCentroid(*input, target_clusters[i].indices, centroid);
                    feature_y.at<float>(i, 0) = centroid.y();
                }
                cv::Mat labels_y, centers_y;
                cv::TermCriteria criteria_y(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 1000, 1.0);
                // Here k=2, because we have base and lateral
                // Set the number of tries to 3, and select the best result
                cv::kmeans(feature_y, 2, labels_y, criteria_y, 3, cv::KMEANS_PP_CENTERS, centers_y);
                // The cluster with the largest y coordinate is the lateral, and the other is the base.
                double y1 = centers_y.at<float>(0, 0);
                double y2 = centers_y.at<float>(1, 0);
                // set y1 to be the larger one, and y2 to be the smaller one.
                if(y1 < y2)
                {
                    std::swap(y1, y2);
                    labels_y = 1 - labels_y; // swap the labels accordingly
                }
                SPDLOG_LOGGER_INFO(logger, "The y coordinate of the lateral and base clusters are: {} and {}", y1, y2);

                pcl::PointIndices::Ptr lateral_indices(new pcl::PointIndices);
                pcl::PointIndices::Ptr base_indices(new pcl::PointIndices);
                for (int i = 0; i < target_clusters.size(); ++i)
                {   
                    SPDLOG_LOGGER_DEBUG(logger, "The y coordinate of the segment {} is: {}, in cluster {}", i, feature_y.at<float>(i, 0), labels_y.at<int>(i, 0));

                    if (labels_y.at<int>(i, 0)==0)
                    {   
                        lateral_indices->indices.reserve(lateral_indices->indices.size() + target_clusters[i].indices.size());
                        lateral_indices->indices.insert(lateral_indices->indices.end(),
                                                        target_clusters[i].indices.begin(),
                                                        target_clusters[i].indices.end());
                    }
                    else
                    {
                        base_indices->indices.reserve(base_indices->indices.size() + target_clusters[i].indices.size());
                        base_indices->indices.insert(base_indices->indices.end(),
                                                     target_clusters[i].indices.begin(),
                                                     target_clusters[i].indices.end());
                    }
                }

                // Extract the lateral and the base clusters
                pcl::ExtractIndices<pcl::PointNormal> extract;
                extract.setInputCloud(input_with_normals);
                if(lateral != nullptr){
                    extract.setIndices(lateral_indices);
                    extract.setNegative(false); // Keep only the specified indices
                    extract.filter(*lateral);
                }
                if(base != nullptr){
                    extract.setIndices(base_indices);
                    extract.setNegative(false); // Keep only the specified indices
                    extract.filter(*base);
                }
            }
            if(lateral != nullptr)
            {
                SPDLOG_LOGGER_INFO(logger, "Saving the lateral({} points) of the cylinder to {}", lateral->size(), path / (prefix + "lateral.ply"));
                SPDLOG_EXEC(logger, spdl::info, pcl::io::savePLYFileASCII(path / (prefix + "lateral.ply"), *lateral));
            }
            if(base != nullptr)
            {
                SPDLOG_LOGGER_INFO(logger, "Saving the base({} points) of the cylinder to {}", base->size(), path / (prefix + "base.ply"));
                SPDLOG_EXEC(logger, spdl::info, pcl::io::savePLYFileASCII(path / (prefix + "base.ply"), *base));
            }
            return input_with_normals;
        } // end normalSegmentation
} // end namespace rn