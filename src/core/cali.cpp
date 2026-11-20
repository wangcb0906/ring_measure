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
#include <opencv2/geometry/2d.hpp>

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
        auto ss = scanLocationToString(loc);
        auto & in = raw_recs[loc];
        if(!in || in->empty())
        {
            SPDLOG_LOGGER_ERROR(logger, "No valid point cloud data for {} scan."
                " Denoising aborted.", str(ss));
            return;
        }
        auto misc = getParam().misc;

        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*in, min_pt, max_pt);
        SPDLOG_LOGGER_INFO(logger, "Reconstructed point cloud from {} scan: {} points.",
                           str(ss), in->size());
        SPDLOG_LOGGER_TRACE(logger,"Bounding box of the reconstruction:");
        SPDLOG_LOGGER_TRACE(logger,"Min point (x_min,y_min,z_min): ({},{},{})",
                            min_pt.x(), min_pt.y(), min_pt.z());
        SPDLOG_LOGGER_TRACE(logger,"Max point (x_max,y_max,z_max): ({},{},{})",
                            max_pt.x(), max_pt.y(), max_pt.z());

        auto downsampled = pcl::make_shared<pcl::PointCloud<PointXYZ>>();
        {
            // Downsample the point cloud using a voxel grid filter
            pcl::VoxelGrid<pcl::PointXYZ> vox;
            vox.setInputCloud(in);
            // set the resample to 1.0 would yield too index overflow.
            float resample = misc.leaf_size_before_EC; // Set downsampling for Euclidean clustering
            vox.setLeafSize(resample, resample, resample); // Set the voxel size (adjust as needed)
            auto cloud_filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            vox.filter(*downsampled);
            SPDLOG_LOGGER_INFO(logger, "Downsampled point cloud for Euclidean clustering:"
                " {} points", downsampled->size());
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

        pcl::PointCloud<pcl::PointXYZ>::Ptr ec_denoised = ecExtract(downsampled, 100, path_den, prefix);
        // write the point cloud to disk
        fs::path file_name = prefix + "original_recon_" + str(ss) + ".ply";
        SPDLOG_LOGGER_DEBUG(logger, "Reconstructed point cloud from {} scan: {} points",
                            str(ss), downsampled->size());
        SPDLOG_EXEC(logger, spdl::debug, savePLY(path_den / file_name, *downsampled));
        SPDLOG_LOGGER_DEBUG(logger, "Saving reconstructed point cloud to {}", path_den / file_name);
        downsampled.reset(); // Release the downsampled cloud to save memory

        SPDLOG_EXEC(logger, spdl::info, savePLY(path_den / (prefix + "recon_denoised.ply"), *ec_denoised));
        SPDLOG_LOGGER_INFO(logger, "Saving the selected EC denoised cluster to {}: {} points",
            path_den / (prefix + "Recon_Denoised.ply"), ec_denoised->size());

        // Further downsample the point cloud to estimate the normals
        pcl::VoxelGrid<pcl::PointXYZ> vox;
        vox.setInputCloud(ec_denoised);
        if(loc == RingMeasure::ScanLocation::Under)
        {   
            auto resample = system.sensor.cam_res * misc.resample_ratio * misc.under_resampling_ratio_for_ne;
            vox.setLeafSize(resample, resample, resample); // Set the voxel size (adjust as needed)
        } else 
        {   
            auto resample = system.sensor.cam_res * misc.resample_ratio * misc.io_resampling_ratio_for_ne;
            vox.setLeafSize(resample, resample, resample);
        }
        vox.filter(*ec_denoised); // using the same buffer to store the downsampled cloud

        auto & lateral = laterals[loc];
        auto & base = bases[loc];
        rec_normals[loc] = normalSegmentation(ec_denoised, path_seg, prefix, lateral, base);

        if(getParam().misc.memory_save)
        {
            rec_normals[loc].reset();
            SPDLOG_LOGGER_INFO(logger, "Memory saving mode is enabled. "
                "The point cloud with normals for scan {} is released.", 
                scanLocationToString(loc));
        }
    }

    void RingCalibration::caliRot(fs::path path_SAC)
    {
        auto params = getParam().primitive;
        auto lateral = laterals[loc];
        if(!lateral || lateral->empty())
        {
            SPDLOG_LOGGER_ERROR(logger, "No valid lateral point cloud data for {} scan. "
                "Rotation calibration aborted.", str(scanLocationToString(loc)));
            return;
        }
          
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
        auto resample = system.sensor.cam_res * getParam().misc.resample_ratio; 
        float min_height = min_pt.y(), max_height = max_pt.y();
        float start_pos = min_height + 0.25 * resample;
        float end = max_height - 0.25 * resample;
        int num_slices =  (max_height - min_height - 0.5 * resample) / resample * 2;
        float sample_dist = resample / 2.0;
        
        // Create a PassThrough filter to extract points within the specified height range
        pcl::PassThrough<pcl::PointNormal> pass;
        pass.setInputCloud(lateral);
        pass.setFilterFieldName("y");
        // create a vector to store the estimations
        std::vector <cv::Point2f> points, points_major;

        pcl::SACSegmentation<pcl::PointNormal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_ELLIPSE3D);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setInputCloud(lateral);
        seg.setDistanceThreshold(params.ellipse_fit_dist_thld);
        seg.setMaxIterations(params.ellipse_fit_max_iter);

        // fit an ellipse if we have enough points.
        // at least 1% of the total points or 120 points
        int threshold = std::max(params.ellipse_fit_num_thld, 
            static_cast<int>(lateral->size()/num_slices*0.9f));
        for (int i=0; i<num_slices; i++)
        {
            pcl::Indices slice_indices;
            float pos = start_pos + (i+0.5)*sample_dist;
            pass.setFilterLimits(pos - 0.5*sample_dist,
                                 pos + 0.5*sample_dist);
            pass.filter(slice_indices);
          
            SPDLOG_LOGGER_TRACE(logger,"Slice {:3d}: at sample height {} has {:5d} points.",
                i, pos, slice_indices.size());
            if(slice_indices.size() > 0)
            {
                // save the slice to disk
                fs::path filename = path_SAC / (std::to_string(i) + ".ply");
                #if SPDLOG_LEVEL_TRACE >= SPDLOG_ACTIVE_LEVEL
                    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_subset =
                        pcl::make_shared<pcl::PointCloud<pcl::PointNormal>>();
                    pcl::copyPointCloud(*lateral, slice_indices, *cloud_subset);
                    SPDLOG_EXEC(logger, spdl::trace, savePLY(filename, *cloud_subset));
                #endif
            }

            if(slice_indices.size() > threshold)
            {    
                pcl::PointIndices::Ptr inliers = pcl::make_shared<pcl::PointIndices>();
                pcl::ModelCoefficients::Ptr coef = pcl::make_shared<pcl::ModelCoefficients>();

                seg.setIndices(std::make_shared<pcl::Indices>(slice_indices));
                seg.segment(*inliers, *coef);

                SPDLOG_LOGGER_TRACE(logger, "--- Estimated 3D Ellipse Parameters ---");
                SPDLOG_LOGGER_TRACE(logger, "Original Point (x,y,z): ({}, {}, {})", 
                                   coef->values[0], coef->values[1], coef->values[2]);
                SPDLOG_LOGGER_TRACE(logger, "Major Axis {:.4f} and minor Axis {:.4f} ",
                                   coef->values[3], coef->values[4]);
                SPDLOG_LOGGER_TRACE(logger, "Normal of the Ellipse (nx,ny,nz): ({}, {}, {})", 
                                   coef->values[5], coef->values[6], coef->values[7]);
                SPDLOG_LOGGER_TRACE(logger, "Direction of the Major Axis (dx,dy,dz): ({}, {}, {})", 
                                   coef->values[8], coef->values[9], coef->values[10]);
                float roundness = coef->values[4] / coef->values[3];
                float angle = std::asin(roundness) * 180 / M_PI;
                SPDLOG_LOGGER_TRACE(logger,"Roundness {}, estimated angle of the table {}",
                                    roundness, angle);
                points.emplace_back(pos, coef->values[4]); // semi-minor axis
                points_major.emplace_back(pos, coef->values[3]); // semi-major axis
            }
        }
        
        SPDLOG_LOGGER_DEBUG(logger,"Points (Radius VS Height) to fit: ");
        for (const auto& pt : points) {
          SPDLOG_LOGGER_DEBUG(logger,"Major Point: ({:4.3f}, {:4.6f})", pt.x, pt.y);
        }
        for (const auto& pt : points_major) {
          SPDLOG_LOGGER_DEBUG(logger,"Minor Point: ({:4.3f}, {:4.6f})", pt.x, pt.y);
        }
        if(points.size() < 2)
        {
            std::string error_msg = "Not enough points to fit a line. Need at least 2 points.";
            SPDLOG_LOGGER_ERROR(logger, error_msg);
            throw std::runtime_error(error_msg);
        }

        // fit line to estimate the angle
        cv::Vec4f line, line_major;
        cv::fitLine (points, line, cv::DIST_HUBER, 0.0, 0.01, 0.01);
        cv::fitLine (points_major, line_major, cv::DIST_HUBER, 0.0, 0.01, 0.01);
        // calculate the angle of the line with respect to the horizontal axis
        SPDLOG_LOGGER_INFO(logger,"[Major] Line coeffs: Normal vector ({}, {}), Point ({},{})", 
            line_major[0], line_major[1], line_major[2], line_major[3]);
        SPDLOG_LOGGER_INFO(logger,"[Minor] Line coeffs: Normal vector ({}, {}), Point ({},{})",
            line[0], line[1], line[2], line[3]);

        // These variables are used in the Python script to plot the profile and fit line.
        const int show = 0;
        std::string path = path_SAC.string();
        // put the break point after ret so path can be initialized before the Python script is called.
        // /py profile.plot_profile($show)

        SPDLOG_LOGGER_INFO(logger, "[Major] Estimated angle of the cone: {} degrees.",
            std::acos(line_major[0]) * 180 / M_PI);
        SPDLOG_LOGGER_INFO(logger, "[Minor] Estimated angle of the cone: {} degrees.",
            std::acos(line[0]) * 180 / M_PI);
        angle[loc][AxisType::MAJOR] = {line_major[0], line_major[1]};
        angle[loc][AxisType::MINOR] = {line[0], line[1]};
        if(params.slices_mad_enabled && PhaseType::VERI == phase_type)
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
            radius[loc][AxisType::MAJOR] = mad_detector.getInlierMean();
            mad_detector.setData(minor_radii);
            mad_detector.fit();
            mad_detector.getInlierMean();
            radius[loc][AxisType::MINOR] = mad_detector.getInlierMean();
            SPDLOG_LOGGER_INFO(logger, "[Major] MAD: Estimated radius of the line fitting: {}.",
                radius[loc][AxisType::MAJOR]);
            SPDLOG_LOGGER_INFO(logger, "[Minor] MAD: Estimated radius of the line fitting: {}.",
                radius[loc][AxisType::MINOR]);
        }
    }

    void
    RingCalibration::caliTrans(fs::path path_SAC)
    {
        auto & base = bases[loc];
        auto & lateral = laterals[loc];
        if(!base || base->empty())
        {
            std::string error_msg = "No valid base point cloud data for " + str(scanLocationToString(loc))
                + " scan. Translation calibration aborted.";
            SPDLOG_LOGGER_ERROR(logger, error_msg);
            throw std::runtime_error(error_msg);
        }
        if(!lateral || lateral->empty())
        {
            std::string error_msg = "No valid lateral point cloud data for " + str(scanLocationToString(loc))
                + " scan. Translation calibration aborted.";
            SPDLOG_LOGGER_ERROR(logger, error_msg);
            throw std::runtime_error(error_msg);
        }
        float diameter = measure.ring.dA;
        auto params = getParam().primitive;

        fs::path file_base = path_SAC / "base.ply";
        fs::path file_lateral = path_SAC / "lateral.ply";
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*lateral, min_pt, max_pt);
        SPDLOG_LOGGER_TRACE(logger,"Bounding box of the lateral:");
        SPDLOG_LOGGER_TRACE(logger,"Min point (x_min,y_min,z_min): ({},{},{})",
            min_pt.x(), min_pt.y(), min_pt.z());
        SPDLOG_LOGGER_TRACE(logger,"Max point (x_max,y_max,z_max): ({},{},{})",
            max_pt.x(), max_pt.y(), max_pt.z());
        pcl::getMinMax3D(*base, min_pt, max_pt);
        SPDLOG_LOGGER_TRACE(logger,"Bounding box of the base:");
        SPDLOG_LOGGER_TRACE(logger,"Min point (x_min,y_min,z_min): ({},{},{})",
            min_pt.x(), min_pt.y(), min_pt.z());
        SPDLOG_LOGGER_TRACE(logger,"Max point (x_max,y_max,z_max): ({},{},{})",
            max_pt.x(), max_pt.y(), max_pt.z());
        SPDLOG_LOGGER_INFO(logger, "Saving the lateral point cloud to {}.", file_lateral);
        SPDLOG_LOGGER_INFO(logger, "Saving the base point cloud to {}.", file_base);
        SPDLOG_EXEC(logger, spdl::info, savePLY(file_lateral, *lateral));
        SPDLOG_EXEC(logger, spdl::info, savePLY(file_base, *base));

        // Setup Cylinder Segmentation
        // Note: The template uses <PointNormal, PointNormal> because the same cloud has both
        pcl::SACSegmentationFromNormals<pcl::PointNormal, pcl::PointNormal> seg;
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
        seg.setProbability(params.cylinder_fit_prob);
        // Pass the SAME cloud as both the points and the normals
        seg.setInputCloud(lateral);
        seg.setInputNormals(lateral); 

        pcl::ModelCoefficients::Ptr cyl_coeff = pcl::make_shared<pcl::ModelCoefficients>();
        pcl::PointIndices::Ptr inliers = pcl::make_shared<pcl::PointIndices>();
        seg.segment(*inliers, *cyl_coeff);
    
        auto cylinder_cloud = pcl::make_shared<pcl::PointCloud<PointNormal>>();
        // Output Results
        if (inliers->indices.empty()) {
            SPDLOG_LOGGER_ERROR(logger, "Could not estimate a cylindrical model,"
                " not enough inliners found.");
        } else {
            pcl::copyPointCloud(*lateral, *inliers, *cylinder_cloud);
            pcl::getMinMax3D(*cylinder_cloud, min_pt, max_pt);
            SPDLOG_LOGGER_TRACE(logger,"Bounding box of the lateral_output:");
            SPDLOG_LOGGER_TRACE(logger,"Min point (x_min,y_min,z_min): ({},{},{})",
                                min_pt.x(), min_pt.y(), min_pt.z());
            SPDLOG_LOGGER_TRACE(logger,"Max point (x_max,y_max,z_max): ({},{},{})",
                                max_pt.x(), max_pt.y(), max_pt.z());
            fs::path filename = path_SAC / "lateral_output.ply";
            SPDLOG_LOGGER_INFO(logger, "Extracted lateral cloud with {}/{} points.",
                cylinder_cloud->size(), lateral->size());
            SPDLOG_EXEC(logger, spdl::debug, savePLY(filename, *cylinder_cloud));
        }
    
        SPDLOG_LOGGER_INFO(logger, "--- Estimated Cylinder Parameters ---");
        SPDLOG_LOGGER_INFO(logger, "Center Point (x,y,z): ({}, {}, {})", 
                           cyl_coeff->values[0], cyl_coeff->values[1], cyl_coeff->values[2]);
        SPDLOG_LOGGER_INFO(logger, "Axis Direction (dx,dy,dz): ({}, {}, {})", 
                           cyl_coeff->values[3], cyl_coeff->values[4], cyl_coeff->values[5]);
        SPDLOG_LOGGER_INFO(logger, "Radius: {:.3f}", cyl_coeff->values[6]);
        float real_diameter = loc == ScanLocation::In ? measure.ring.dI / 2.0 : measure.ring.dA / 2.0;
        SPDLOG_LOGGER_INFO(logger, "The real value of the radius is: {:.3f}", real_diameter);
        
        // radius, axis, center
        cyl_fit[loc] = std::make_tuple(cyl_coeff->values[6],
             Eigen::Vector3f(cyl_coeff->values[3], cyl_coeff->values[4], cyl_coeff->values[5]),
             Eigen::Vector3f(cyl_coeff->values[0], cyl_coeff->values[1], cyl_coeff->values[2])
        );

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

        auto plane_coeff = pcl::make_shared<pcl::ModelCoefficients>();
        auto plane_inliers = pcl::make_shared<pcl::PointIndices>();
        seg.segment(*plane_inliers, *plane_coeff);

        auto plane_cloud = pcl::make_shared<pcl::PointCloud<PointNormal>>();
        // Output Results
        if (plane_inliers->indices.empty()) {
            SPDLOG_LOGGER_INFO(logger, "Could not estimate a plane model.");
        } else {
            pcl::copyPointCloud(*base, *plane_inliers, *plane_cloud);
            pcl::getMinMax3D(*plane_cloud, min_pt, max_pt);
            SPDLOG_LOGGER_TRACE(logger,"Bounding box of the plane_cloud:");
            SPDLOG_LOGGER_TRACE(logger,"Min point (x_min,y_min,z_min): ({},{},{})",
                                min_pt.x(), min_pt.y(), min_pt.z());
            SPDLOG_LOGGER_TRACE(logger,"Max point (x_max,y_max,z_max): ({},{},{})",
                                max_pt.x(), max_pt.y(), max_pt.z());
            fs::path filename = path_SAC / "plane_output.ply";
            SPDLOG_LOGGER_INFO(logger, "Extracted plane cloud with {}/{} points.",
                               plane_cloud->size(), base->size());
            SPDLOG_EXEC(logger, spdl::debug, savePLY(filename, *plane_cloud));

            SPDLOG_LOGGER_INFO(logger, "--- Plane Coefficients ---");
            SPDLOG_LOGGER_INFO(logger, "Normal Vector: ({}, {}, {}), Distance from origin: {}", 
                               plane_coeff->values[0], 
                               plane_coeff->values[1], 
                               plane_coeff->values[2], 
                               plane_coeff->values[3]);
            float real_height = (loc == ScanLocation::Under ? 0.0 : measure.ring.h)
                + system.robot.table_h;
            SPDLOG_LOGGER_INFO(logger, "The real value of the height is: {}", real_height);
            // height, normal
            base_fit[loc] = std::make_tuple(plane_coeff->values[3],
                Eigen::Vector3f(plane_coeff->values[0], plane_coeff->values[1], plane_coeff->values[2])
            );
        }
        #if SPDLOG_LEVEL_TRACE >= ACTIVE_LOG_LEVEL
            // draw the histogram of the y coordinates
            std::vector<float> y_coords;
            y_coords.reserve(base->size());
            for (const auto& point : base->points) {
                y_coords.push_back(point.y);
            }
            std::string path = path_SAC.string();
            const int show = 0;
            // /py hist.plot_hist($show)
        #endif

        // dump the cylinder before and after SAC estimation
        SPDLOG_LOGGER_TRACE(logger, "Saving the cylinder point cloud to {}.",
            path_SAC / "cylinder.ply");
        SPDLOG_LOGGER_DEBUG(logger, "Saving the segmented cylinder point cloud to {}.",
            path_SAC / "cylinder_output.ply");
        SPDLOG_EXEC(logger, spdl::trace, savePLY(path_SAC / "cylinder.ply",
            *base + *lateral));
        SPDLOG_EXEC(logger, spdl::debug, savePLY(path_SAC / "cylinder_output.ply",
            *cylinder_cloud + *plane_cloud));
    }
 
    void RingCalibration::
    calibration(const rm::ScanLocation& scan,
                const fs::path& path)
    {   
        if(!isReady()) {
            std::string error_msg = "Calibration is not ready. " 
                "Please ensure that the system and"
                " measure configurations are properly initialized.";
            SPDLOG_LOGGER_ERROR(logger, error_msg);
            throw std::runtime_error(error_msg);
        }
        auto ss = scanLocationToString(scan);
        setScanLoc(scan);
        setPhaseType(rm::PhaseType::CALI);
        fs::path path_den, path_seg, path_SAC;
        #if SPDLOG_LEVEL_WARN > SPDLOG_ACTIVE_LEVEL
            path_den = path / (str(ss) + "_cali_denoise");
            CREATE_DIR(logger, path_den);
            path_seg = path / (str(ss) + "_cali_segment");
            CREATE_DIR(logger, path_seg);
            path_SAC = path / (str(ss) + "_cali_slices");
            CREATE_DIR(logger, path_SAC);
        #endif
        auto time_start = std::chrono::high_resolution_clock::now();
        SPDLOG_LOGGER_INFO(logger, std::string(60, '='));
        SPDLOG_LOGGER_INFO(logger, "Setting the location to '{}'.", ss);
        SPDLOG_LOGGER_INFO(logger, std::string(60, '='));
        SPDLOG_LOGGER_INFO(logger, "Starting calibration for '{}' scan.", ss);

        setProcessType(rm::ProcessType::ROT);
        SPDLOG_LOGGER_INFO(logger, "Starting the reconstruction for '{}' scan.", ss);
        reconstruct(path_den);
        SPDLOG_LOGGER_INFO(logger, "Reconstruction for '{}' scan completed.", ss);

        SPDLOG_LOGGER_INFO(logger, "Starting the denoising for '{}' scan.", ss);
        bases[loc].reset();
        laterals[loc].reset(new pcl::PointCloud<pcl::PointNormal>());
        denoise(path_den, path_seg, true);
        SPDLOG_LOGGER_INFO(logger, "Denoising for '{}' scan completed.", ss);

        SPDLOG_LOGGER_INFO(logger, "Starting the rotation calibration for '{}' scan.", ss);
        caliRot(path_SAC);
        SPDLOG_LOGGER_INFO(logger, "Rotation calibration for '{}' scan completed.", ss);

        // Update the setting of system and measure after the rotation calibration.
        auto &direction = angle[loc][AxisType::MINOR];
        float angle = std::atan2(direction[1], direction[0]) * 180 / M_PI;
        switch (scan)
        {
            case rm::ScanLocation::Out:
            case rm::ScanLocation::Under:
                system.robot.scans[loc].angle_corr -= angle;
                break;
            case rm::ScanLocation::In:
                system.robot.scans[loc].angle_corr += angle;
                break;
            default: break;
        }

        bases[loc].reset(new pcl::PointCloud<pcl::PointNormal>());
        laterals[loc].reset(new pcl::PointCloud<pcl::PointNormal>());
        SPDLOG_LOGGER_INFO(logger, "Starting the translation calibration for '{}' scan.", ss);
        setProcessType(rm::ProcessType::TRANS);
        reconstruct(path_den);
        denoise(path_den, path_seg, true);
        caliTrans(path_SAC);
        SPDLOG_LOGGER_INFO(logger, "Translation calibration for '{}' scan completed.", ss);
        if(param.misc.memory_save)
        {
            bases[loc].reset();
            laterals[loc].reset();
        }

        // Update the setting of system and measure after the translation calibration.
        auto pos_est = rm::Point2f(std::get<0>(cyl_fit[loc]), std::get<0>(base_fit[loc]));
        switch (scan)
        {   
            case rm::ScanLocation::Out:
            {
                // Compute the real radius of the ring. radius = ring.dA/2
                // Compute the real postion of the plane. pos = table_h + ring.h
                rm::Point2f out_real (measure.ring.dA/2.0f,
                    system.robot.table_h + measure.ring.h);
                rm::Point2f out_correction = pos_est - out_real;
                system.robot.scans[loc].pos_corr.x += out_correction.x;
                system.robot.scans[loc].pos_corr.y -= out_correction.y;
                break;
            }
            case rm::ScanLocation::In:
            {
                // Compute the real radius of the ring. radius = ring.dI/2
                // Compute the real postion of the plane. pos = table_h + ring.h
                rm::Point2f in_real (measure.ring.dI/2.0f,
                    system.robot.table_h + measure.ring.h);
                rm::Point2f in_correction = pos_est - in_real;
                system.robot.scans[loc].pos_corr -= in_correction;
                break;
            }
            case rm::ScanLocation::Under:
            {
                // Compute the real radius of the ring. radius = ring.dA/2
                // Compute the real postion of the plane. pos = table_h
                rm::Point2f under_real (measure.ring.dA/2.0f, system.robot.table_h);
                rm::Point2f under_correction = pos_est - under_real;
                system.robot.scans[loc].pos_corr -= under_correction;
                break;
            }
            default: break;
        }
        auto time_end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>
            (time_end - time_start).count();
        SPDLOG_LOGGER_INFO(logger, "Calibration for '{}' scan completed in {} ms.\n", ss, duration);
    }

    void RingCalibration::
    verification(const rm::ScanLocation& scan,
                 const fs::path& path,
                 bool profile)
    {
        if(!isReady()) {
            std::string error_msg = "Verification is not ready. " 
                "Please ensure that the system and measure configurations are properly initialized.";
            SPDLOG_LOGGER_ERROR(logger, error_msg);
            throw std::runtime_error(error_msg);
        }
        auto ss = scanLocationToString(scan);
        setScanLoc(scan);
        setPhaseType(rm::PhaseType::VERI);
        fs::path path_den, path_seg, path_SAC;
        #if SPDLOG_LEVEL_WARN > SPDLOG_ACTIVE_LEVEL
            path_den = path / (str(ss) + "_veri_denoise");
            CREATE_DIR(logger, path_den);
            path_seg = path / (str(ss) + "_veri_segment");
            CREATE_DIR(logger, path_seg);
            path_SAC = path / (str(ss) + "_veri_slices");
            CREATE_DIR(logger, path_SAC);
        #endif
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
        
        if(param.misc.memory_save)
        {
            bases[loc].reset();
            laterals[loc].reset();
        }

        SPDLOG_LOGGER_INFO(logger, "Cylinder verification for '{}' scan completed.", ss);
        SPDLOG_LOGGER_INFO(logger, "Calibration for '{}' scan completed.\n", ss);
    }
}

