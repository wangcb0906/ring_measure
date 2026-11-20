#include "visualization.hpp"
// To make SPDLOG_LOGGER_TRACE work with fs::path
#include <spdlog/fmt/std.h>

namespace RingMeasure
{   
#if SPDLOG_LEVEL_DEBUG >= ACTIVE_LOG_LEVEL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorizeClusters(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        const std::vector<pcl::PointIndices>& cluster_indices) 
    {
        // Create the output colored point cloud
        PcPtr<pcl::PointXYZRGB> colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (const auto& cluster : cluster_indices) {
            // Generate a random color for the current cluster
            std::uint8_t r = static_cast<std::uint8_t>(std::rand() % 256);
            std::uint8_t g = static_cast<std::uint8_t>(std::rand() % 256);
            std::uint8_t b = static_cast<std::uint8_t>(std::rand() % 256);
    
            // Assign points from this cluster to the colored cloud
            for (const auto& index : cluster.indices) {
                pcl::PointXYZRGB point;
                point.x = input_cloud->points[index].x;
                point.y = input_cloud->points[index].y;
                point.z = input_cloud->points[index].z;
                point.r = r;
                point.g = g;
                point.b = b;
                colored_cloud->push_back(point);
            }
        }
    
        colored_cloud->width = colored_cloud->size();
        colored_cloud->height = 1;
        colored_cloud->is_dense = true;
    
        return colored_cloud;
    }

    void visualizeCurvature(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                            float thld,
                            std::filesystem::path ply_path,
                            const std::shared_ptr<spdlog::logger> &logger)
    {
        assert(cloud->size() && "All points have NaN or Inf curvature values.");
        auto colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        colored_cloud->resize(cloud->size());

        if(std::abs(thld) < 1e-6f){
            SPDLOG_LOGGER_DEBUG(logger, "Max-min colorize the curvature point cloud.");
            auto [min_it, max_it] = std::minmax_element(
                cloud->begin(), cloud->end(),
                [](const pcl::PointNormal& a, const pcl::PointNormal& b) {
                    return a.curvature < b.curvature;
                }
            );
            float min_curv = min_it->curvature;
            float max_curv = max_it->curvature;
            float curv_range = (max_curv == min_curv) ? 1.0f : (max_curv - min_curv);
            SPDLOG_LOGGER_DEBUG(logger, "Curvature range: min = {}, max = {}, range = {}",
                min_curv, max_curv, curv_range);
    
            for(size_t i = 0; i < cloud->size(); ++i){
                const auto& src = cloud->points[i];
                auto& dest = colored_cloud->points[i];
                dest.x = src.x;
                dest.y = src.y;
                dest.z = src.z;
                float normalized_curv = (src.curvature - min_curv) / curv_range;
                // Map to a color gradient (e.g., Blue = Flat/Low, Red = Sharp/High)
                // Low curvature -> Blue (B=255), High curvature -> Red (R=255)
                // save original curvature values for debugging
                // visualize the curvature using a color gradient
                // Value : 0.0       | 0.2       | 0.4  | 0.6          | 0.8    | 1.0
                // Color : dark blue | cyan-blue | cyan | yellow-green | orange | red
                dest.r = static_cast<uint8_t>(normalized_curv * 255);
                dest.g = static_cast<uint8_t>((1.0f - std::abs(normalized_curv - 0.5f) * 2.0f) * 255);
                dest.b = static_cast<uint8_t>((1.0f - normalized_curv) * 255);
            }
        } else {
            SPDLOG_LOGGER_DEBUG(logger, "Using provided curvature threshold:"
                " {} to bianrize the point cloud.", thld);
            for(size_t i = 0; i < cloud->size(); ++i){
                const auto& src = cloud->points[i];
                auto& dest = colored_cloud->points[i];
                dest.x = src.x;
                dest.y = src.y;
                dest.z = src.z;
                if(src.curvature < thld){
                    // Low curvature -> Black
                    dest.r = 0;
                    dest.g = 0;
                    dest.b = 0;
                } else {
                    // High curvature -> White
                    dest.r = 255;
                    dest.g = 255;
                    dest.b = 255;
                }
            }
        
        }
        SPDLOG_LOGGER_INFO(logger, "Saving the visualized curvature to {}.", ply_path);
        SPDLOG_EXEC(logger, spdlog::level::trace, savePLY(ply_path, *colored_cloud));
    }

    void visualizeNormal(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                         std::filesystem::path ply_path,
                         const std::shared_ptr<spdlog::logger> &logger)
    {
        auto colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        colored_cloud->resize(cloud->size());

        SPDLOG_LOGGER_DEBUG(logger, "Max-min colorize the normal point cloud.");
        auto [min_it, max_it] = std::minmax_element(
            cloud->begin(), cloud->end(),
            [](const pcl::PointNormal& a, const pcl::PointNormal& b) {
                return std::abs(a.normal_y) < std::abs(b.normal_y);
            }
        );
        float min_normal = std::abs(min_it->normal_y);
        float max_normal = std::abs(max_it->normal_y);
        float normal_range = (max_normal == min_normal) ? 1.0f : (max_normal - min_normal);
        SPDLOG_LOGGER_DEBUG(logger, "Normal range: min = {}, max = {}, range = {}",
            min_normal, max_normal, normal_range);
    
        for(size_t i = 0; i < cloud->size(); ++i){
            const auto& src = cloud->points[i];
            auto& dest = colored_cloud->points[i];
            dest.x = src.x;
            dest.y = src.y;
            dest.z = src.z;
            float normalized_normal = (std::abs(src.normal_y) - min_normal) / normal_range;
            // Low normal -> Blue , High normal -> Red
            // Map to a color gradient (e.g., Blue = orthogonal to y axis, Red = parallel to y axis)
            // save original normal values for debugging
            // visualize the normal using a color gradient
            // Value : 0.0       | 0.2       | 0.4  | 0.6          | 0.8    | 1.0
            // Color : dark blue | cyan-blue | cyan | yellow-green | orange | red
            dest.r = static_cast<uint8_t>(normalized_normal * 255);
            dest.g = static_cast<uint8_t>((1.0f - std::abs(normalized_normal - 0.5f) * 2.0f) * 255);
            dest.b = static_cast<uint8_t>((1.0f - normalized_normal) * 255);
        }

        SPDLOG_LOGGER_INFO(logger, "Saving the visualized max-min normalized normal to {}.", ply_path);
        SPDLOG_EXEC(logger, spdlog::level::trace, savePLY(ply_path, *colored_cloud));
    }

    void
    visualizeObliquity(const RingCalibration& cali, fs::path ply_path)
    {   
        auto bases = cali.getBases();
        EnumArray<ScanLocation, PcPtr<pcl::PointXYZRGB>> clouds;
        for(int i = 0; i < static_cast<int>(ScanLocation::NUM); ++i)
        {
            if(!bases[i] || bases[i]->empty())
            {
                SPDLOG_LOGGER_WARN(cali.getLogger(), "Base point cloud for scan {} is empty. "
                    "Skipping obliquity visualization for this scan.", i);
                continue;
            }

            // Find min and max Y values to normalize the gradient range
            auto [min_it, max_it] = std::minmax_element(bases[i]->points.begin(), bases[i]->points.end(),
                 [](const pcl::PointNormal& a, const pcl::PointNormal& b) {
                        return a.y < b.y;
                    });
            float min_y = min_it->y;
            float max_y = max_it->y;
            if (min_y == max_y) max_y = min_y + 1.0f; // Prevent division by zero

            clouds[i].reset(new pcl::PointCloud<pcl::PointXYZRGB>());
            clouds[i]->points.resize(bases[i]->points.size());
            for(size_t j = 0; j < bases[i]->points.size(); ++j){
                const auto& src = bases[i]->points[j];
                auto& dest = clouds[i]->points[j];
                // Copy XYZ coordinates
                dest.x = src.x;
                dest.y = src.y;
                dest.z = src.z;
                float normalized_y = (src.y - min_y) / (max_y - min_y);
                dest.r = static_cast<uint8_t>(normalized_y * 255);
                dest.g = static_cast<uint8_t>((1.0f - std::abs(normalized_y - 0.5f) * 2.0f) * 255);
                dest.b = static_cast<uint8_t>((1.0f - normalized_y) * 255);
            }
        }
        SPDLOG_LOGGER_TRACE(cali.getLogger(), "Saving the obliquity visualization to {}.", ply_path);
        SPDLOG_EXEC(cali.getLogger(), spdlog::level::trace, savePLY(ply_path.string(),
                    *clouds[0] + *clouds[1] + *clouds[2]));
    }
#endif
}