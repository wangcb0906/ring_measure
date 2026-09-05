#include <iostream>
#include <vector>
// read the image
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp> 
#include <opencv2/highgui.hpp> 
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <spdlog/spdlog.h>

#include "cali.hpp"

void RingMeasure::RingCalibration::reconstruct(const fs::path& path_)
{
    auto IDX = static_cast<size_t>(scan_loc);
    int is_under = (scan_loc == RingMeasure::ScanLocation::Under) ? -1 : 1;
    const std::string &scan = (measure.img.path / measure.img.imgs[IDX]).string();
    
    // read the scan image and convert the img to float
    cv::Mat img_ = cv::imread(scan, cv::IMREAD_GRAYSCALE | cv::IMREAD_ANYDEPTH);
    // check if the image is loaded successfully
    if (img_.empty())
    {
        SPDLOG_LOGGER_ERROR(logger, "Error: Could not open or find the image {} ", scan);
        return;
    }

    cv::Mat img;
    img_.convertTo(img, CV_32F);

    // convert the image to float real world distance with the sensor calibration
    img *= system.sensor.z_pitch/1.0e5;
    img += system.sensor.z_oft/1.0e5;
    // Subtract the offset w.r.t. the mounting height of the camera
    img -= system.sensor.mount_height;

    // After all these conversion, the invalid pixels
    // invalid = 0 * z_pitch + z_oft - mount_height 
    //         = 0 - 524.288 - 980 = 1504.288 mm

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
    std::vector<double> vec = {
        alpha, -beta, pos.x,
        beta,  alpha, pos.y};
    cv::Mat R(2, 3, CV_64F, vec.data()); 
    // Create a vector for the x coordinates of a scan line
    // For the scans from outside and inside, the x coordinates are from -360 to 360,
    // For the scans from the bottom, the x coordinates should be reversed.
    std::vector<float> x;
    x.resize(system.sensor.scan_res);

    float aux = static_cast<float>(system.sensor.valid_len) / static_cast<float>(system.sensor.scan_res);
    for (int i=0; i<system.sensor.scan_res; ++i)
    {
        x[i] = aux * (system.sensor.scan_res/2l - i) * is_under;
    }

    raw_recs[IDX] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // Processing each scan line to reconstruct the 3D point cloud
    SPDLOG_LOGGER_INFO(logger,"Processing scan image {}", scan);
    for (int idx=0; idx<360; ++idx){
        // create a auxiliary point cloud to store the reconstruction for a line
        pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // create a auxiliary point cloud to extract the 2D point cloud
        std::vector<cv::Point2f> points;

        // Extract only the valid points from the scan line.
        for (int i=0; i<system.sensor.scan_res; ++i)
        {
            if (img.at<float>(idx, i) > smallest_value){
                points.emplace_back(x[i], img.at<float>(idx, i));
            }
        }
        
        // Transform the points from camera coord to world coord
        if(points.empty()){
            SPDLOG_LOGGER_WARN(logger, "No valid points found in scan line {:3d}.", idx);
            continue;
        }

        cv::transform(points, points, R);
        for (const auto& p : points)
        {
            line_cloud->emplace_back(p.x, p.y, 0.0f);
        }
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(idx / 180.0 * M_PI, Eigen::Vector3f(0, 1.0, 0)));
        pcl::transformPointCloud(*line_cloud, *line_cloud, transform);

        // Add the line cloud to the final reconstruction
        *raw_recs[IDX] += *line_cloud;

        // /py line.plot_line($idx, $show)
        const int show = 0; // Set show = 1 to plot the interactive one
        // Convert the points to pcl point cloud
        const std::string path = path_.string();
        if( (scan_loc == RingMeasure::ScanLocation::Under)
            && phase_type == RingMeasure::PhaseType::VERI){
            int a = 0; // placeholder
            // SPDLOG_LOGGER_INFO(spdlog::default_logger(), "Drawing scan line {} : {:3d} with {} valid points.", scanLocationToString(scan_loc), idx, points.size());
        }
    }
}