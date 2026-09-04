// #define WITHOUT_NUMPY
// #include <matplotlibcpp.h>
#include "cali.hpp"
#include "log.hpp"
#include "3rdparty/mad.hpp"
#include <spdlog/spdlog.h>
#include <spdlog/fmt/std.h>

#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h> //resampling
#include <pcl/filters/passthrough.h> //passthrough
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h> // pcl::getMinMax3D

#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace {

    // pcl::PolygonMesh generateSolidEllipseMesh(const Eigen::VectorXf& coeffs, int steps = 36) {
    //     pcl::PolygonMesh mesh;
    //     if (coeffs.size() < 11) return mesh;
    
    //     // 1. Unpack coefficients
    //     Eigen::Vector3d center(coeffs[0], coeffs[1], coeffs[2]);
    //     double r_u = coeffs[3], r_v = coeffs[4];
    //     Eigen::Vector3d normal = Eigen::Vector3d(coeffs[5], coeffs[6], coeffs[7]).normalized();
    //     Eigen::Vector3d u_dir = Eigen::Vector3d(coeffs[8], coeffs[9], coeffs[10]).normalized();
    //     Eigen::Vector3d v_dir = normal.cross(u_dir).normalized();
    
    //     // 2. Build vertices: Center point is Index 0, perimeter points follow
    //     pcl::PointCloud<pcl::PointXYZ> cloud;
    //     cloud.push_back(pcl::PointXYZ(center.x(), center.y(), center.z())); // Index 0
    
    //     for (int i = 0; i < steps; ++i) {
    //         double theta = 2.0 * M_PI * i / steps;
    //         Eigen::Vector3d pt = center + (u_dir * r_u * std::cos(theta)) + (v_dir * r_v * std::sin(theta));
    //         cloud.push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z())); // Indices 1 to steps
    //     }
    
    //     pcl::toPCLPointCloud2(cloud, mesh.cloud);
    
    //     // 3. Create triangles linking center (0) to the perimeter ring
    //     std::vector<pcl::Vertices> polygons;
    //     for (int i = 1; i <= steps; ++i) {
    //         int next_i = (i % steps) + 1;
    //         pcl::Vertices tri;
    //         tri.vertices = {0, i, next_i};
    //         polygons.push_back(tri);
    //     }
    
    //     mesh.polygons = polygons;
    //     return mesh;
    //     }
}


namespace RingMeasure {

    namespace fs=std::filesystem;
    namespace spdl=spdlog::level;
    using str = std::string;

    // This function segment the base and lateral of the cylinder.
    void
    RingCalibration::denoise(
        fs::path path_den,
        fs::path path_seg,
        bool free_raw)
    {
        auto ss = scanLocationToString(scan_loc);
        auto out = raw_recs[loc];
        auto misc = getParam().misc;

        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*out, min_pt, max_pt);
        SPDLOG_LOGGER_INFO(logger, "Reconstructed point cloud from {} scan: {} points.", str(ss), out->size());
        SPDLOG_LOGGER_TRACE(logger,"Bounding box of the reconstruction:");
        SPDLOG_LOGGER_TRACE(logger,"Min point (x_min,y_min,z_min): ({},{},{})", min_pt.x(), min_pt.y(), min_pt.z());
        SPDLOG_LOGGER_TRACE(logger,"Max point (x_max,y_max,z_max): ({},{},{})", max_pt.x(), max_pt.y(), max_pt.z());
        {
            // Downsample the point cloud using a voxel grid filter
            pcl::VoxelGrid<pcl::PointXYZ> vox;
            vox.setInputCloud(out);
            // set the resample to 1.0 would yield too index overflow.
            float resample = misc.leaf_size_before_EC; // Set downsampling for Euclidean clustering
            vox.setLeafSize(resample, resample, resample); // Set the voxel size (adjust as needed)
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            vox.filter(*cloud_filtered);
            out = cloud_filtered; // Use the downsampled cloud for further processing
            SPDLOG_LOGGER_INFO(logger, "Downsampled point cloud for Euclidean clustering: {} points", out->size());
        }

        if(free_raw)
        {
            raw_recs[loc].reset();
        }
        

        std::string prefix;
        if(phase_type != rm::PhaseType::VERI)
        {
            prefix = std::string(processTypeToString(process_type)) + "_";
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr out_denoised = ecExtract(out, 100, path_den, prefix);
        // write the point cloud to disk
        fs::path file_name = prefix + "original_recon_" + str(ss) + ".ply";
        SPDLOG_LOGGER_INFO(logger, "Reconstructed point cloud from {} scan: {} points", str(ss), out->size());
        SPDLOG_LOGGER_INFO(logger, "Saving reconstructed point cloud to {}", path_den / file_name);
        SPDLOG_EXEC(logger, spdl::info, pcl::io::savePLYFileASCII(path_den / file_name, *out));
        SPDLOG_LOGGER_INFO(logger, "Saving the selected EC denoised cluster to {}: {} points", path_den / (prefix + "Recon_Denoised.ply"), out_denoised->size());
        SPDLOG_EXEC(logger, spdl::info, pcl::io::savePLYFileASCII(path_den / (prefix + "Recon_Denoised.ply"), *out_denoised));
      
        // Further downsample the point cloud to estimate the normals
        pcl::VoxelGrid<pcl::PointXYZ> vox;
        vox.setInputCloud(out_denoised);
        if(scan_loc == RingMeasure::ScanLocation::Under)
        {   
            auto resample = camera_resolution * misc.resample_ratio * misc.under_resampling_ratio_for_ne;
            vox.setLeafSize(resample, resample, resample); // Set the voxel size (adjust as needed)
        } else 
        {   
            auto resample = camera_resolution * misc.resample_ratio * misc.io_resampling_ratio_for_ne;
            vox.setLeafSize(resample, resample, resample);
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        vox.filter(*out_denoised); // using the same buffer to store the downsampled cloud
      
        auto & lateral = laterals[loc];
        auto & base = bases[loc];
        rec_normals[loc] = normalSegmentation(out_denoised, path_seg, prefix, lateral, base);
    }

    void RingCalibration::caliRot(fs::path path_SAC)
    {
        auto params = getParam().primitive;
        auto lateral = laterals[loc];
          
        // extract the bounding box of the lateral
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*lateral, min_pt, max_pt);
        
        SPDLOG_LOGGER_TRACE(logger,"Bounding box of the lateral:");
        SPDLOG_LOGGER_TRACE(logger,"Min point (x_min,y_min,z_min): ({},{},{})", min_pt.x(), min_pt.y(), min_pt.z());
        SPDLOG_LOGGER_TRACE(logger,"Max point (x_max,y_max,z_max): ({},{},{})", max_pt.x(), max_pt.y(), max_pt.z());
        
        // Configure the horizontal slicing for each cut.
        // We start to cut the cone from the bottom with an offset of 0.25*resample and end at the top 
        // with an offset of 0.25*resample. 
        // The number of slices is determined by the height of the cone.
        // Then we recalculate the sample distance between each slice.
        auto resample = camera_resolution * getParam().misc.resample_ratio; 
        float min_height = min_pt.y(), max_height = max_pt.y();
        float start_pos = min_height + 0.25 * resample;
        float end = max_height - 0.25 * resample;
        int num_slices = static_cast<int>((max_height - min_height - 0.5 * resample) / resample * 2);
        float sample_dist = resample / 2.0;
        
        // Create a PassThrough filter to extract points within the specified height range
        pcl::PassThrough<pcl::PointNormal> pass;
        pass.setInputCloud(lateral);
        pass.setFilterFieldName("y");
        // Create a vector to store the point cloud for each slice
        std::vector <pcl::PointCloud<pcl::PointNormal>::Ptr> slices(num_slices);
        for(auto &cloud : slices) {
          cloud.reset(new pcl::PointCloud<pcl::PointNormal>());
        }
        // create a vector to store the estimations
        std::vector <cv::Point2f> points, points_major;
        bool flag = std::filesystem::create_directories(path_SAC);
        if(!flag & !std::filesystem::exists(path_SAC))
        {
          SPDLOG_LOGGER_ERROR(logger, "Failed to create directory: {}", path_SAC);
          exit(EXIT_FAILURE);
        }

        for (int i=0; i<num_slices; i++)
        {
            float pos = start_pos + (i+0.5)*sample_dist;;
            pass.setFilterLimits(pos - 0.5*sample_dist,
                                 pos + 0.5*sample_dist);
            pass.filter(*slices[i]);
          
            SPDLOG_LOGGER_TRACE(logger,"Slice {:3d}: at sample height {} has {:5d} points.", i, pos, slices[i]->size());
            if(slices[i]->size() > 0)
            {
                // save the slice to disk
                fs::path filename = path_SAC / (std::to_string(i) + ".ply");
                SPDLOG_EXEC(logger, spdl::trace, pcl::io::savePLYFileASCII(filename, *slices[i]));
            }
            // fit an ellipse if we have enough points.
            // at least 1% of the total points or 120 points
            int threshold = std::max(params.ellipse_fit_num_thld, static_cast<int>(lateral->size()/num_slices*0.9f));

            if(slices[i]->size() > threshold)
            {
                pcl::SACSegmentation<pcl::PointNormal> seg;
                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_ELLIPSE3D);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setInputCloud(slices[i]);
                seg.setDistanceThreshold(params.ellipse_fit_dist_thld);
                seg.setMaxIterations(params.ellipse_fit_max_iter);
    
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
                pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            
                seg.segment(*inliers, *coefficients);
                SPDLOG_LOGGER_TRACE(logger, "--- Estimated 3D Ellipse Parameters ---");
                SPDLOG_LOGGER_TRACE(logger, "Original Point (x,y,z): ({}, {}, {})", 
                                   coefficients->values[0], coefficients->values[1], coefficients->values[2]);
                SPDLOG_LOGGER_TRACE(logger, "Major Axis {:.4f} and minor Axis {:.4f} ", coefficients->values[3], coefficients->values[4]);
                SPDLOG_LOGGER_TRACE(logger, "Normal of the Ellipse (nx,ny,nz): ({}, {}, {})", 
                                   coefficients->values[5], coefficients->values[6], coefficients->values[7]);
                SPDLOG_LOGGER_TRACE(logger, "Direction of the Major Axis (dx,dy,dz): ({}, {}, {})", 
                                   coefficients->values[8], coefficients->values[9], coefficients->values[10]);
                float roundness = coefficients->values[4] / coefficients->values[3];
                float angle = std::asin(roundness) * 180 / M_PI;
                SPDLOG_LOGGER_TRACE(logger,"Roundness {}, estimated angle of the table {}",  roundness, angle);
                points.emplace_back(pos, coefficients->values[4]); // semi-minor axis
                points_major.emplace_back(pos, coefficients->values[3]); // semi-major axis
            }
        }
        
        SPDLOG_LOGGER_DEBUG(logger,"Points (Radius VS Height) to fit: ");
        for (const auto& pt : points) {
          SPDLOG_LOGGER_DEBUG(logger,"Point: ({:4.3f}, {:4.6f})", pt.x, pt.y);
        }
        if(points.size() < 2)
        {
          SPDLOG_LOGGER_ERROR(logger, "Not enough points to fit a line. Need at least 2 points.");
          exit(EXIT_FAILURE);
        }

        // fit line to estimate the angle
        cv::Vec4f line, line_major;
        cv::fitLine (points, line, cv::DIST_HUBER, 0.0, 0.01, 0.01);
        cv::fitLine (points_major, line_major, cv::DIST_HUBER, 0.0, 0.01, 0.01);
        // calculate the angle of the line with respect to the horizontal axis
        SPDLOG_LOGGER_INFO(logger,"[Major] Line coefficients: Normal vector ({}, {}), Point ({},{})", line_major[0], line_major[1], line_major[2], line_major[3]);
        SPDLOG_LOGGER_INFO(logger,"[Minor] Line coefficients: Normal vector ({}, {}), Point ({},{})", line[0], line[1], line[2], line[3]);

        // These variables are used in the Python script to plot the profile and fit line.
        const int show = 0;
        std::string path = path_SAC.string();
        // put the break point after ret so path can be initialized before the Python script is called.
        // /py profile.plot_profile($show)

        angle_est[loc] = std::atan2(line[1], line[0]) * 180 / M_PI;
        SPDLOG_LOGGER_INFO(logger, "[Major] Estimated angle of the cone: {} degrees.", std::atan2(line_major[1], line_major[0]) * 180 / M_PI);
        SPDLOG_LOGGER_INFO(logger, "[Minor] Estimated angle of the cone: {} degrees.", angle_est[loc]);
        
        if(params.slices_mad_enabled && rm::PhaseType::VERI == phase_type)
        {
            SPDLOG_LOGGER_DEBUG(logger, "Radius and angle estimation from line fitting enabled for verification.");
            // See mad.hpp to tune the threshold for outlier detection
            MADDetector<float> mad_detector(params.mad_thld);
            std::vector<float> major_radii, minor_radii;
            for (const auto& pt : points_major) {
                major_radii.push_back(pt.y);
            }
            for (const auto& pt : points) {
                minor_radii.push_back(pt.y);
            }
            mad_detector.setData(major_radii);
            mad_detector.fit();
            mad_detector.getInlierMean();
            radius_eval[loc][static_cast<size_t>(AxisType::MAJOR)] = mad_detector.getInlierMean();
            mad_detector.setData(minor_radii);
            mad_detector.fit();
            mad_detector.getInlierMean();
            radius_eval[loc][static_cast<size_t>(AxisType::MINOR)] = mad_detector.getInlierMean();
            SPDLOG_LOGGER_DEBUG(logger, "[Major] MAD: Estimated radius of the line fitting: {}.", radius_eval[loc][static_cast<size_t>(AxisType::MAJOR)]);
            SPDLOG_LOGGER_DEBUG(logger, "[Minor] MAD: Estimated radius of the line fitting: {}.", radius_eval[loc][static_cast<size_t>(AxisType::MINOR)]);
        }
    }

    void
    RingCalibration::caliTrans(fs::path path_SAC)
    {
        auto base = bases[loc];
        auto lateral = laterals[loc];
        float diameter = measure.ring.dA;
        auto params = getParam().primitive;

        fs::path file_base = path_SAC / "base.ply";
        fs::path file_lateral = path_SAC / "lateral.ply";
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*lateral, min_pt, max_pt);
        SPDLOG_LOGGER_TRACE(logger,"Bounding box of the lateral:");
        SPDLOG_LOGGER_TRACE(logger,"Min point (x_min,y_min,z_min): ({},{},{})", min_pt.x(), min_pt.y(), min_pt.z());
        SPDLOG_LOGGER_TRACE(logger,"Max point (x_max,y_max,z_max): ({},{},{})", max_pt.x(), max_pt.y(), max_pt.z());
        pcl::getMinMax3D(*base, min_pt, max_pt);
        SPDLOG_LOGGER_TRACE(logger,"Bounding box of the base:");
        SPDLOG_LOGGER_TRACE(logger,"Min point (x_min,y_min,z_min): ({},{},{})", min_pt.x(), min_pt.y(), min_pt.z());
        SPDLOG_LOGGER_TRACE(logger,"Max point (x_max,y_max,z_max): ({},{},{})", max_pt.x(), max_pt.y(), max_pt.z());
        SPDLOG_LOGGER_INFO(logger, "Saving the lateral point cloud to {}.", file_lateral);
        SPDLOG_EXEC(logger, spdl::info, pcl::io::savePLYFileASCII(file_lateral, *lateral));
        SPDLOG_LOGGER_INFO(logger, "Saving the base point cloud to {}.", file_base);
        SPDLOG_EXEC(logger, spdl::info, pcl::io::savePLYFileASCII(file_base, *base));

        // Setup Cylinder Segmentation
        // Note: The template uses <PointNormal, PointNormal> because the same cloud has both
        pcl::SACSegmentationFromNormals<pcl::PointNormal, pcl::PointNormal> seg;
        pcl::ModelCoefficients::Ptr cylinder_coeff(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(params.cylinder_fit_normal_weight);
        seg.setMaxIterations(params.cylinder_fit_max_iter);
        // The maximum allowable Euclidean distance (in meters) from a 3D point
        // to the outer surface of the candidate cylinder for that point to be considered an inlier
        seg.setDistanceThreshold(params.cylinder_fit_dist_thld); 
        // seg.setDistanceThreshold(0.1); 
        seg.setRadiusLimits(10.0, diameter/2.0*1.5);
    
        // Pass the SAME cloud as both the points and the normals
        seg.setInputCloud(lateral);
        seg.setInputNormals(lateral); 
        
        seg.segment(*inliers, *cylinder_coeff);
    
        pcl::PointCloud<pcl::PointNormal>::Ptr cylinder_cloud(new pcl::PointCloud<pcl::PointNormal>());
        // Output Results
        if (inliers->indices.empty()) {
            SPDLOG_LOGGER_ERROR(logger, "Could not estimate a cylindrical model, not enough inliners found.");
        } else {
            // Assuming 'cloud' is your input cloud, and 'inliers' contains the cylinder indices:
            pcl::ExtractIndices<pcl::PointNormal> extract;
            
            extract.setInputCloud(lateral);
            extract.setIndices(inliers);
            extract.setNegative(false); // false = keep ONLY the cylinder inliers
            extract.filter(*cylinder_cloud);

            pcl::getMinMax3D(*cylinder_cloud, min_pt, max_pt);
            SPDLOG_LOGGER_TRACE(logger,"Bounding box of the lateral_output:");
            SPDLOG_LOGGER_TRACE(logger,"Min point (x_min,y_min,z_min): ({},{},{})", min_pt.x(), min_pt.y(), min_pt.z());
            SPDLOG_LOGGER_TRACE(logger,"Max point (x_max,y_max,z_max): ({},{},{})", max_pt.x(), max_pt.y(), max_pt.z());
            fs::path filename = path_SAC / "lateral_output.ply";
            SPDLOG_LOGGER_DEBUG(logger, "Extracted lateral cloud with {}/{} points.", cylinder_cloud->size(), lateral->size());
            SPDLOG_EXEC(logger, spdl::debug, pcl::io::savePLYFileASCII(filename, *cylinder_cloud));
        }
    
        SPDLOG_LOGGER_INFO(logger, "--- Estimated Cylinder Parameters ---");
        SPDLOG_LOGGER_INFO(logger, "Center Point (x,y,z): ({}, {}, {})", 
                           cylinder_coeff->values[0], cylinder_coeff->values[1], cylinder_coeff->values[2]);
        SPDLOG_LOGGER_INFO(logger, "Axis Direction (dx,dy,dz): ({}, {}, {})", 
                           cylinder_coeff->values[3], cylinder_coeff->values[4], cylinder_coeff->values[5]);
        SPDLOG_LOGGER_INFO(logger, "Radius: {:.3f}", cylinder_coeff->values[6]);
        
        // radius, axis, center
        cyl_fit[loc] = std::make_tuple(cylinder_coeff->values[6],
             Eigen::Vector3f(cylinder_coeff->values[3], cylinder_coeff->values[4], cylinder_coeff->values[5]),
             Eigen::Vector3f(cylinder_coeff->values[0], cylinder_coeff->values[1], cylinder_coeff->values[2])
        );

        // Save the estimated position to pos_est
        pos_est[loc].x = cylinder_coeff->values[6];

        // Estimate the position of the mount table.
        // Always set the table at the height of 982mm.
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setRadiusLimits(0.0, std::numeric_limits<double>::max());
        seg.setDistanceThreshold(params.plane_fit_dist_thld);
        seg.setNormalDistanceWeight(params.plane_fit_normal_weight);
        seg.setMaxIterations(params.plane_fit_max_iter);
        seg.setOptimizeCoefficients(true);
        seg.setInputCloud(base);
        seg.setInputNormals(base);

        pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);

        seg.segment(*plane_inliers, *plane_coeff);
        pos_est[loc].y = plane_coeff->values[3];

        pcl::PointCloud<pcl::PointNormal>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointNormal>());
        // Output Results
        if (plane_inliers->indices.empty()) {
            SPDLOG_LOGGER_INFO(logger, "Could not estimate a plane model.");
        } else {
            // Assuming 'cloud' is your input cloud, and 'inliers' contains the cylinder indices:
            pcl::ExtractIndices<pcl::PointNormal> extract;
            
            extract.setInputCloud(base);
            extract.setIndices(plane_inliers);
            extract.setNegative(false); // false = keep ONLY the cylinder inliers
            extract.filter(*plane_cloud);
            
            pcl::getMinMax3D(*plane_cloud, min_pt, max_pt);
            SPDLOG_LOGGER_TRACE(logger,"Bounding box of the plane_cloud:");
            SPDLOG_LOGGER_TRACE(logger,"Min point (x_min,y_min,z_min): ({},{},{})", min_pt.x(), min_pt.y(), min_pt.z());
            SPDLOG_LOGGER_TRACE(logger,"Max point (x_max,y_max,z_max): ({},{},{})", max_pt.x(), max_pt.y(), max_pt.z());
            fs::path filename = path_SAC / "plane_output.ply";
            SPDLOG_LOGGER_DEBUG(logger, "Extracted plane cloud with {}/{} points.", plane_cloud->size(), base->size());
            SPDLOG_EXEC(logger, spdl::debug, pcl::io::savePLYFileASCII(filename, *plane_cloud));
        }

        SPDLOG_LOGGER_INFO(logger, "--- Plane Coefficients ---");
        SPDLOG_LOGGER_INFO(logger, "Normal Vector: ({}, {}, {}), Distance from origin: {}", 
                           plane_coeff->values[0], 
                           plane_coeff->values[1], 
                           plane_coeff->values[2], 
                           plane_coeff->values[3]);
        // height, normal
        base_fit[loc] = std::make_tuple(plane_coeff->values[3],
             Eigen::Vector3f(plane_coeff->values[0], plane_coeff->values[1], plane_coeff->values[2])
        );

        // draw the histogram of the y coordinates
        std::vector<float> y_coords;
        y_coords.reserve(base->size());
        for (const auto& point : base->points) {
            y_coords.push_back(point.y);
        }
        std::string path = path_SAC.string();
        const int show = 0;
        // /py hist.plot_hist($show)

        // dump the cylinder before and after SAC estimation
        SPDLOG_LOGGER_INFO(logger, "Saving the cylinder point cloud to {}.", path_SAC / "cylinder.ply");
        SPDLOG_EXEC(logger, spdl::info, pcl::io::savePLYFileASCII(path_SAC / "cylinder.ply", *base + *lateral));
        SPDLOG_LOGGER_DEBUG(logger, "Saving the segmented cylinder point cloud to {}.", path_SAC / "cylinder_output.ply");
        SPDLOG_EXEC(logger, spdl::debug, pcl::io::savePLYFileASCII(path_SAC / "cylinder_output.ply", *cylinder_cloud + *plane_cloud));
    }
 
    void RingCalibration::
    calibration(const rm::ScanLocation& scan,
                const fs::path& path)
    {   
        if(!isReady()) {
            SPDLOG_LOGGER_ERROR(logger, "Calibration is not ready.\nPlease ensure that the system ans measure configurations are properly initialized.");
        }
        auto ss = scanLocationToString(scan);
        setScanLoc(scan);
        auto path_den = path / (str(ss) + "_cali_denoise");
        CREATE_DIR(logger, path_den);
        auto path_seg = path / (str(ss) + "_cali_segment");
        CREATE_DIR(logger, path_seg);
        auto path_SAC = path / (str(ss) + "_cali_slices");
        CREATE_DIR(logger, path_SAC);

        SPDLOG_LOGGER_INFO(logger, std::string(60, '='));
        SPDLOG_LOGGER_INFO(logger, "Setting the location to '{}'.", ss);
        SPDLOG_LOGGER_INFO(logger, std::string(60, '='));
        SPDLOG_LOGGER_INFO(logger, "Starting calibration for '{}' scan.", ss);

        setProcessType(rm::ProcessType::ROT);
        SPDLOG_LOGGER_INFO(logger, "Starting the reconstruction for '{}' scan.", ss);
        reconstruct(path_den);
        SPDLOG_LOGGER_INFO(logger, "Reconstruction for '{}' scan completed.", ss);

        SPDLOG_LOGGER_INFO(logger, "Starting the denoising for '{}' scan.", ss);
        laterals[loc].reset(new pcl::PointCloud<pcl::PointNormal>());
        denoise(path_den, path_seg, true);
        SPDLOG_LOGGER_INFO(logger, "Denoising for '{}' scan completed.", ss);

        SPDLOG_LOGGER_INFO(logger, "Starting the rotation calibration for '{}' scan.", ss);
        caliRot(path_SAC);
        SPDLOG_LOGGER_INFO(logger, "Rotation calibration for '{}' scan completed.", ss);

        // Update the setting of system and measure after the rotation calibration.
        switch (scan)
        {
            case rm::ScanLocation::Out:
            case rm::ScanLocation::Under:
                system.robot.scans[loc].angle_corr -= angle_est[loc];
                break;
            case rm::ScanLocation::In:
                system.robot.scans[loc].angle_corr += angle_est[loc];
                break;
            default: break;
        }

        bases[loc].reset(new pcl::PointCloud<pcl::PointNormal>());
        SPDLOG_LOGGER_INFO(logger, "Starting the translation calibration for '{}' scan.", ss);
        setProcessType(rm::ProcessType::TRANS);
        reconstruct(path_den);
        denoise(path_den, path_seg, true);
        caliTrans(path_SAC);
        SPDLOG_LOGGER_INFO(logger, "Translation calibration for '{}' scan completed.", ss);

        // Update the setting of system and measure after the translation calibration.
        switch (scan)
        {
            case rm::ScanLocation::Out:
            {
                // Compute the real radius of the ring. radius = ring.dA/2
                // Compute the real postion of the plane. pos = table_h + ring.h
                rm::Point2f out_real (measure.ring.dA/2.0f,
                    system.robot.table_h + measure.ring.h - system.robot.wedge_h);
                rm::Point2f out_correction = pos_est[loc] - out_real;
                system.robot.scans[loc].pos_corr.x += out_correction.x;
                system.robot.scans[loc].pos_corr.y -= out_correction.y;
                break;
            }
            case rm::ScanLocation::In:
            {
                // Compute the real radius of the ring. radius = ring.dI/2
                // Compute the real postion of the plane. pos = table_h + ring.h
                rm::Point2f in_real (measure.ring.dI/2.0f,
                    system.robot.table_h + measure.ring.h - system.robot.wedge_h);
                rm::Point2f in_correction = pos_est[loc] - in_real;
                system.robot.scans[loc].pos_corr -= in_correction;
                break;
            }
            case rm::ScanLocation::Under:
            {
                // Compute the real radius of the ring. radius = ring.dA/2
                // Compute the real postion of the plane. pos = table_h
                rm::Point2f under_real (measure.ring.dA/2.0f,
                    system.robot.table_h - system.robot.wedge_h);
                rm::Point2f under_correction = pos_est[loc] - under_real;
                system.robot.scans[loc].pos_corr -= under_correction;
                break;
            }
            default: break;
        }
        SPDLOG_LOGGER_INFO(logger, "Calibration for '{}' scan completed.\n", ss);
    }

    void RingCalibration::
    verification(const rm::ScanLocation& scan,
                 const fs::path& path,
                 bool profile)
    {
        if(!isReady()) {
            SPDLOG_LOGGER_ERROR(logger, "Calibration is not ready.\nPlease ensure that the system ans measure configurations are properly initialized.");
        }
        auto ss = scanLocationToString(scan);
        setScanLoc(scan);
        auto path_den = path / (str(ss) + "_veri_denoise");
        CREATE_DIR(logger, path_den);
        auto path_seg = path / (str(ss) + "_veri_segment");
        CREATE_DIR(logger, path_seg);
        auto path_SAC = path / (str(ss) + "_veri_slices");
        CREATE_DIR(logger, path_SAC);
        SPDLOG_LOGGER_INFO(logger, std::string(60, '='));
        SPDLOG_LOGGER_INFO(logger, "Setting the location to '{}'.", ss);
        SPDLOG_LOGGER_INFO(logger, std::string(60, '='));
        SPDLOG_LOGGER_INFO(logger, "Starting verification for '{}' scan.", ss);

        SPDLOG_LOGGER_INFO(logger, "Starting the verification for '{}' scan.", ss);
        reconstruct(path_den);
        SPDLOG_LOGGER_INFO(logger, "Reconstruction for '{}' scan completed.", ss);
        SPDLOG_LOGGER_INFO(logger, "Starting the denoising for '{}' scan.", ss);
        laterals[loc].reset(new pcl::PointCloud<pcl::PointNormal>());
        bases[loc].reset(new pcl::PointCloud<pcl::PointNormal>());
        denoise(path_den, path_seg, true);
        SPDLOG_LOGGER_INFO(logger, "Denoising for '{}' scan completed.", ss);

        if(profile)
        {
            SPDLOG_LOGGER_INFO(logger, "Starting the profile estimation for '{}' scan.", ss);
            caliRot(path_SAC);
            SPDLOG_LOGGER_INFO(logger, "Profile estimation for '{}' scan completed.", ss);
        }
        else
        {
            SPDLOG_LOGGER_INFO(logger, "Skipping the profile estimation for '{}' scan.", ss);
        }

        SPDLOG_LOGGER_INFO(logger, "Starting the cylinder verification for '{}' scan.", ss);
        caliTrans(path_SAC);
        SPDLOG_LOGGER_INFO(logger, "Cylinder verification for '{}' scan completed.", ss);
        

        SPDLOG_LOGGER_INFO(logger, "Calibration for '{}' scan completed.\n", ss);
    }
}

