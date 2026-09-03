#include <vector>
#include <cassert>

#include <iomanip> // to set the precision in inicpp
#include <inicpp.h>


#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
// To make SPDLOG_LOGGER_TRACE work with fs::path
#include <spdlog/fmt/std.h>

#include "visualization.hpp"

// #include <pcl/io/ply_io.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/segmentation/region_growing.h>
// #include <pcl/segmentation/impl/region_growing.hpp>
// #include <pcl/surface/mls.h>
// #include <pcl/surface/gp3.h>

#include <boost/multi_array.hpp>

int
main (int argc, char* argv[])
{
    namespace fs = std::filesystem;
    namespace rm = RingMeasure;
    // logger configuration
    auto logger=spdlog::default_logger();
    logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%l] [%s:%#] %v");

    if(argc != 2) 
    {
        SPDLOG_LOGGER_ERROR(logger, "Invalid number of arguments. Usage: {} <measure_config_file/folder>", argv[0]);
        exit(EXIT_FAILURE);
    }

    fs::path config_path = argv[1];

    std::vector<fs::path> calis;
    if (fs::exists(config_path) && fs::is_directory(config_path)) {
        // Populate vector directly using directory iterator range
        SPDLOG_LOGGER_INFO(logger, "Processing the measure configuration folder: {}", config_path);
        for (auto const& entry : fs::directory_iterator(config_path)) {
            if (entry.is_regular_file()) {
                calis.push_back(entry.path());
            }
        }
    } else {
        SPDLOG_LOGGER_ERROR(logger, "Directory {} does not exist.", config_path);
        exit(EXIT_FAILURE);
    }

    auto system_cfg_file = config_path / "system.ini";

    SPDLOG_LOGGER_WARN(logger, "Starting reconstruction...");
    fs::path log_path("logs/measure_cali");
    CREATE_DIR(logger, log_path);

    // Create a 3D grid where EVERY cell is a dynamic std::vector
    boost::multi_array<rm::RingCalibration::ScanMetricsArray, 3> eval_scan(
        boost::extents[static_cast<size_t>(rm::ScanLocation::NUM)]
        [calis.size()][calis.size()]
    );
    boost::multi_array<rm::RingCalibration::ReconMetricsArray, 2> eval_recon(
        boost::extents[calis.size()][calis.size()]
    );

    for (size_t cali_idx = 0; cali_idx < calis.size(); ++cali_idx) {
    
        rm::RingCalibration cali;
        cali.setPhaseType(rm::PhaseType::VERI);
        const auto& cali_cfg_file = calis[cali_idx];
        logger=spdlog::basic_logger_mt("MeasureCali", log_path / ("[" + std::to_string(cali_idx) + "]" + cali_cfg_file.stem().string() + ".log"), true);
        logger->set_level(spdlog::level::trace);
        logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%l] [%s:%#] %v");
        // wrapper the PCl logger with time and level information
        auto timeLevelCallback = createPclLogHandler(logger);
        pcl::console::Logger::getInstance().setCallback(timeLevelCallback);
        cali.setLogger(logger);

        assert(fs::exists(cali_cfg_file) && "The calibration configuration file does not exist.");
        assert(fs::exists(system_cfg_file) && "The system configuration file does not exist.");
        SPDLOG_LOGGER_INFO(logger, "Loading configuration from {}.", system_cfg_file);
        rm::SystemCfg sys;
        rm::loadSystemConfig(system_cfg_file, sys);
        SPDLOG_LOGGER_INFO(logger, "Loaded configuration from {}.", system_cfg_file);
        // Read the bianary calibration configuration file
        // calibration : system_XXXX, substr(7)
        fs::path cali_cfg_bin = cali_cfg_file.parent_path() / "calibrated" / cali_cfg_file.stem().string();
        assert(fs::exists(cali_cfg_bin) && "The binary cali config file does not exist.");
        std::ifstream in_file(cali_cfg_bin, std::ios::binary);
        std::array<RingMeasure::ScanCfg, static_cast<size_t>(rm::ScanLocation::NUM)> &
            cfgs_read = sys.robot.scans;
        in_file.read(
                reinterpret_cast<char*>(cfgs_read.data()),
                cfgs_read.size() * sizeof(RingMeasure::ScanCfg)
                );
        in_file.close();
        cali.setSystem(sys);
        fs::path root_out_path = "result/measure_cali_" + cali_cfg_file.stem().string();
        CREATE_DIR(logger, root_out_path);
        fs::path path = root_out_path / (cali_cfg_file.stem().string());
        auto start_time = std::chrono::high_resolution_clock::now();
        // Start to evaluate the calibration results

        for (size_t eval_idx = 0; eval_idx < calis.size(); ++eval_idx)
        {
            const auto& ring = calis[eval_idx];
            fs::path ring_path = root_out_path / (ring.stem().string());
            CREATE_DIR(logger, ring_path);
            SPDLOG_LOGGER_CRITICAL(logger, "Processing Ring {} with config {}", ring.filename(),  cali_cfg_file.filename());
            rm::MeasureCfg measure;
            rm::loadMeasureConfig(ring, measure);
            cali.setMeasure(measure);
            constexpr std::array<rm::ScanLocation, static_cast<size_t>(rm::ScanLocation::NUM)> scan_locs = {
                rm::ScanLocation::In,
                rm::ScanLocation::Out,
                rm::ScanLocation::Under
            };
            for (const auto& scan : scan_locs)
            {
                cali.verification(scan, ring_path, true);
                auto loc = static_cast<size_t>(scan);
                // /py
                eval_scan[loc][cali_idx][eval_idx] = cali.scanEval();
                if(ring.string().find("_3_1") != std::string::npos)
                {
                    eval_scan[loc][cali_idx][eval_idx][static_cast<size_t>(rm::RingCalibration::ScanMetrics::H_PLANE_FIT)] -= 10;
                }
            }

            eval_recon[cali_idx][eval_idx] = cali.reconEval();
            std::string path_str = log_path.string();
            float da = measure.ring.dA;
            float di = measure.ring.dI;
            float h = measure.ring.h;
            float table_h = sys.robot.table_h;
            // /py eval.plot_eval($da, $di, $h, $table_h)
            int a = 0; // placeholder
        }
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        SPDLOG_LOGGER_WARN(logger, "Total execution time: {} ms", duration.count());
        SPDLOG_LOGGER_INFO(logger, "Finished processing the evaluation {}.\n\n\n", cali_cfg_file);
    }
    
    //     // write the reconstructed point clouds to PLY files for visualization
    //     *rec_out += *rec_in;
    //     *rec_out += *rec_under;
    //     // Downsample the point cloud using a voxel grid filter
    //     pcl::VoxelGrid<pcl::PointNormal> vox;
    //     vox.setInputCloud(rec_out);
    //     int camera_resolution = measure.ring.dA * M_PI / 360.0f;
    //     float resample = camera_resolution * 0.75f;
    //     vox.setLeafSize(resample, resample, resample); // Set the voxel size (adjust as needed)
    //     pcl::PointCloud<pcl::PointNormal>::Ptr out (new pcl::PointCloud<pcl::PointNormal>);
    //     vox.filter(*out);
    //     SPDLOG_LOGGER_INFO(logger, "Downsampled reconstruction has {} points.", out->size());
    //     SPDLOG_LOGGER_INFO(logger, "Writing the reconstruction to {}", path / "reconstructed.ply");
    //     SPDLOG_EXEC(logger, spdlog::level::info, pcl::io::savePLYFileASCII(path / "reconstructed.ply", *out));

    //     pcl::RegionGrowing<pcl::PointNormal, pcl::PointNormal> reg;
    //     pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
    //     reg.setResidualThreshold (camera_resolution * 0.3f);
    //     reg.setMinClusterSize (1);
    //     reg.setMaxClusterSize (out->size());
    //     tree->setInputCloud (out);
    //     reg.setSearchMethod (tree);
    //     reg.setNumberOfNeighbours (8);
    //     reg.setInputCloud (out);
    //     reg.setInputNormals (out);
    //     reg.setSmoothnessThreshold (15.0 / 18.0 * M_PI);
    //     reg.setCurvatureThreshold (0.05); // in
    //     std::vector <pcl::PointIndices> segmentation_indices;
    //     reg.extract (segmentation_indices);
    //     std::sort(segmentation_indices.begin(), segmentation_indices.end(),
    //              [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
    //              return a.indices.size() > b.indices.size(); // Sort in descending order of cluster size
    //     });
    //     SPDLOG_LOGGER_TRACE(logger, "Write the {} clusters of out segmented to: {}", segmentation_indices.size(), path / "recon_seg");
    //     SPDLOG_EXEC(logger, spdlog::level::trace, RingMeasure::writeSegmentation<pcl::PointNormal>(out, segmentation_indices, path / "recon_seg", "_RecSeg", 10));
    //     pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    //     SPDLOG_LOGGER_TRACE(logger, "Saving the colored segmented point cloud to {}", path / "noisy_recon.ply");
    //     SPDLOG_EXEC(logger, spdlog::level::trace, pcl::io::savePLYFileASCII(path / "noisy_recon.ply", *colored_cloud));

    //     // Extract the segs with large size
    //     auto split = std::partition(segmentation_indices.begin(), segmentation_indices.end(),
    //         [&out](const pcl::PointIndices& indices) {
    //             return indices.indices.size() > out->size() * 0.1f; // Adjust the threshold as needed
    //         });
    //     int num_of_segs = std::distance(split, segmentation_indices.end());
    //     SPDLOG_LOGGER_TRACE(logger, "Number of clusters with size less than 10% of the total points: {}", num_of_segs);
    //     cv::Mat feature(num_of_segs, 1, CV_32F); // the feature vector is the size of the clusters
    //     int k = 3; // number of clusters in k-means
    //     int idx = 0;
    //     for (auto it = split; it != segmentation_indices.end(); ++it, ++idx) {
    //         feature.at<float>(idx, 0) = it->indices.size();
    //     }
    //     cv::Mat labels, centers;
    //     cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 1000, 1.0);
    //     cv::kmeans(feature, k, labels, criteria, 3, cv::KMEANS_PP_CENTERS, centers);
    //     cv::Point min_loc;
    //     cv::minMaxLoc(centers, nullptr, nullptr, &min_loc, nullptr);
    //     int discard = min_loc.x;
    //     SPDLOG_LOGGER_TRACE(logger, "Discarding the cluster with label: {}", discard);
    //     pcl::PointIndices::Ptr discard_indices(new pcl::PointIndices); ;
    //     int offset = split - segmentation_indices.begin();
    //     for (int i = offset; i < num_of_segs; ++i){
    //         SPDLOG_LOGGER_TRACE(logger, "Cluster {:2d} has size {:4d} and is assigned to cluster {}", i, segmentation_indices[i].indices.size(), labels.at<int>(i - offset, 0));
    //         if (labels.at<int>(i - offset, 0) == discard)
    //         {
    //             discard_indices->indices.insert(discard_indices->indices.end(), segmentation_indices[i].indices.begin(), segmentation_indices[i].indices.end());
    //         }
    //     }
    //     // Extract the lateral and the base clusters
    //     pcl::PointCloud<pcl::PointNormal>::Ptr rec(new pcl::PointCloud<pcl::PointNormal>);
    //     pcl::ExtractIndices<pcl::PointNormal> extract;
    //     extract.setInputCloud(out);
    //     extract.setIndices(discard_indices);
    //     extract.setNegative(true);
    //     extract.filter(*rec);
    //     SPDLOG_LOGGER_INFO(logger, "Writing the final reconstruction to {}", path / "recon_pc.ply");
    //     SPDLOG_EXEC(logger, spdlog::level::info, pcl::io::savePLYFileASCII(path / "recon_pc.ply", *rec));

    //     pcl::search::KdTree<pcl::PointNormal>::Ptr gp3_tree(new pcl::search::KdTree<pcl::PointNormal>());
    //     gp3_tree->setInputCloud(rec);
    
    //     // Initialize GP3 reconstructor
    //     pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    //     pcl::PolygonMesh mesh;
    
    //     // Set search radius (max distance between connected vertices)
    //     // Keep this slightly larger than your point spacing, but small enough to preserve holes
    //     gp3.setSearchRadius(resample*2.0);
    //     // Set triangulation parameters
    //     gp3.setMu(2.5);                         // Multiplier for nearest-neighbor distance
    //     gp3.setMaximumNearestNeighbors(100);    // Max neighbors considered per point
    //     gp3.setMaximumSurfaceAngle(M_PI / 4);   // 45 deg max angle between normals
    //     gp3.setMinimumAngle(M_PI / 18);         // 10 deg min triangle angle
    //     gp3.setMaximumAngle(2 * M_PI / 3);      // 120 deg max triangle angle
    //     gp3.setNormalConsistency(false);        // Ensure consistent face normals
    
    //     // Connect input cloud and search tree to GP3
    //     gp3.setInputCloud(rec);
    //     gp3.setSearchMethod(gp3_tree);
    
    //     // Reconstruct mesh
    //     gp3.reconstruct(mesh);

    //     SPDLOG_LOGGER_INFO(logger, "Writing the reconstructed mesh to {}", path / "recon_mesh.ply");
    //     SPDLOG_EXEC(logger, spdlog::level::info, pcl::io::savePLYFile(path / "recon_mesh.ply", mesh));

    // flush all the logs and terminate the threads safely.
    spdlog::shutdown();
    return (0);
}