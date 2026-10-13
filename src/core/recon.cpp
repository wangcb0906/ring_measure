#include <iostream>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <spdlog/spdlog.h>

#include <tiffio.h>

#include "cali.hpp"

void RingMeasure::RingCalibration::reconstruct(const fs::path& path_)
{
    auto IDX = static_cast<size_t>(scan_loc);
    int is_under = (scan_loc == RingMeasure::ScanLocation::Under) ? -1 : 1;
    const std::string &scan = (measure.img.path / measure.img.imgs[IDX]).string();

    TIFF* tif = TIFFOpen(scan.c_str(), "r");
    if (!tif) {
        throw std::runtime_error("Failed to open TIFF file: " + scan);
    }

    // In the real world transformation, the camera first have a 
    // translation w.r.t the world coord origin, then a rotation w.r.t. 
    // to camera coord orgin. When we reverse the transformation,
    // we also inverse the order.

    // create the 2D rotation matrix to transfer from camera coord to world corrd
    float angle = is_under * (90.0f 
                             + system.robot.scans[IDX].view_angle
                             + system.robot.scans[IDX].angle_corr);
    double alpha = std::cos(angle*M_PI/180.0);
    double beta = std::sin(angle*M_PI/180.0);
    auto pos = measure.robot_pos[IDX] + system.robot.scans[IDX].pos_corr;

    Eigen::Matrix4f T_affine = Eigen::Matrix4f::Identity();
    T_affine(0, 0) = alpha;  T_affine(0, 1) = -beta; T_affine(0, 3) = pos.x;
    T_affine(1, 0) = beta;   T_affine(1, 1) = alpha;  T_affine(1, 3) = pos.y;
    
    const size_t max_res = system.sensor.scan_res;
    std::vector<float> x;
    x.resize(max_res);

    float aux = static_cast<float>(system.sensor.valid_len) / static_cast<float>(system.sensor.scan_res);
    for (int i=0; i<system.sensor.scan_res; ++i)
    {
        x[i] = aux * (system.sensor.scan_res/2l - i) * is_under;
    }

    raw_recs[IDX] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // Processing each scan line to reconstruct the 3D point cloud
    SPDLOG_LOGGER_INFO(logger,"Processing scan image {}", scan);
    // create a auxiliary point cloud to store the reconstruction of a line
    pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    line_cloud->reserve(max_res);
    // create a auxiliary vector to store the depth of a line
    std::vector<uint16_t> scan_buffer(max_res);
    for (int idx=0; idx<360; ++idx){
        line_cloud->clear();

        if (TIFFReadScanline(tif, scan_buffer.data(), idx) < 0) {
            SPDLOG_LOGGER_WARN(logger, "Failed to read scan line {:3d} from TIFF.", idx);
            continue;
        }

        for (size_t i = 0; i < max_res; ++i) {
            if(scan_buffer[i] > 0)
            {
                float raw_y = static_cast<float>(scan_buffer[i]) * pitch_scale
                              + pitch_offset;
                line_cloud->emplace_back(x[i], raw_y, 0.0f);
            }
        }

        if (line_cloud->empty()) {
            SPDLOG_LOGGER_WARN(logger, "No valid points found in scan line {:3d}.", idx);
            continue;
        }

        // Compute sweeping Y-axis rotation
        Eigen::Affine3f T_y = Eigen::Affine3f::Identity();
        T_y.rotate(Eigen::AngleAxisf(idx / 180.0 * M_PI, Eigen::Vector3f(0, 1.0, 0)));

        #if SPDLOG_LEVEL_TRACE >= ACTIVE_LOG_LEVEL
        {
            pcl::transformPointCloud(*line_cloud, *line_cloud, T_affine);

            const int show = 1; // Set show = 1 to plot the interactive one
            if( 
                // (scan_loc == RingMeasure::ScanLocation::Under) &&
                phase_type == RingMeasure::PhaseType::VERI){
                float ra = measure.ring.dA/2;
                float ri = measure.ring.dI/2;
                float h = measure.ring.h;
                float table_h = system.robot.table_h;
                const std::string path = path_.string();
                std::vector<float> cloud_x;
                std::vector<float> cloud_y;
                cloud_x.reserve(max_res);
                cloud_y.reserve(max_res);
                for (const auto& pt : line_cloud->points)
                {
                    cloud_x.push_back(pt.x);
                    cloud_y.push_back(pt.y);
                }
                // disable the multi-threading first, see ring_cali.cpp
                // Set show = 1 to plot the interactive one
                // /py line.plot_line($idx, $show, $ra, $ri, $h, $table_h)
                // SPDLOG_LOGGER_INFO(spdlog::default_logger(), "Drawing scan line {} : {:3d} with {} valid to file {}.", scanLocationToString(scan_loc), idx, cloud_x.size(), path);
            }
            pcl::transformPointCloud(*line_cloud, *line_cloud, T_y);
        }

        #else
        {
            // Combine two transformations
            Eigen::Matrix4f T_total = T_y.matrix() * T_affine;
            pcl::transformPointCloud(*line_cloud, *line_cloud, T_total);
        }
        #endif
        
        // Add the line cloud to the final reconstruction
        *raw_recs[IDX] += *line_cloud;
    }

    TIFFClose(tif);
}