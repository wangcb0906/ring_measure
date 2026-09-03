#include <vector>
#include <cassert>
#include <iomanip> // to set the precision in inicpp
#include <inicpp.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
// To make SPDLOG_LOGGER_TRACE work with fs::path
#include <spdlog/fmt/std.h>
#include "visualization.hpp"

#if defined(RING_MEASURE_CALI_ONLY) && defined(RING_MEASURE_VERIFY_ONLY)
    #error "Cannot define both RING_MEASURE_CALI_ONLY and RING_MEASURE_VERIFY_ONLY!"
#endif

int
main (int argc, char* argv[])
{
    namespace fs = std::filesystem;
    namespace rm = RingMeasure;

    rm::RingCalibration cali;

    // logger configuration
    auto logger=spdlog::default_logger();
    logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%l] [%s:%#] %v");

    if(argc != 2) 
    {
        SPDLOG_LOGGER_ERROR(logger, "Invalid number of arguments. Usage: {} <measure_config_file/folder>", argv[0]);
        exit(EXIT_FAILURE);
    }

    fs::path path_arg = argv[1];
    if (!fs::exists(path_arg)) {
        SPDLOG_LOGGER_ERROR(logger, "File {} does not exist.", path_arg);
        exit(EXIT_FAILURE);
    }

    bool is_file = fs::is_regular_file(path_arg);
    // Read the configuration of the robot
    fs::path single_cfg_file;
    fs::path config_path;

    if(is_file)
    {
        SPDLOG_LOGGER_INFO(logger, "Processing the measure configuration file: {}", path_arg);
        config_path = path_arg.parent_path();
        single_cfg_file = path_arg.filename();
    }
    else
    {
        SPDLOG_LOGGER_INFO(logger, "Processing the measure configuration folder: {}", path_arg);
        config_path = path_arg;
    }
    fs::path cali_path = config_path / "calibrated";
    if(!fs::exists(cali_path))
    {
      CREATE_DIR(logger, cali_path);
    }
    fs::path system_cfg_file = "system.ini";
    system_cfg_file = config_path / system_cfg_file;

    for (const auto& measure_cfg_file : fs::directory_iterator(config_path)) {
        // Exclude the system configuration file
        if (measure_cfg_file.path().filename() == system_cfg_file.filename()) {
            continue; 
        }
        // Exclude any folder inside the config folder
        if(measure_cfg_file.is_directory()){
            SPDLOG_LOGGER_INFO(logger, "Ignoring directory: {}\n\n\n", measure_cfg_file.path());
            continue;
        }
        // If the argument is a file, only process that specific file
        if (is_file && measure_cfg_file.path().filename() != single_cfg_file) {
            continue;
        }

        auto start_time = std::chrono::high_resolution_clock::now();

        std::string ring = measure_cfg_file.path().stem().string();
        logger=spdlog::basic_logger_mt(ring, "logs/" + ring + ".log", true);
        logger->set_level(spdlog::level::trace);
        logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%l] [%s:%#] %v");
        // wrapper the PCl logger with time and level information
        auto timeLevelCallback = createPclLogHandler(logger);
        pcl::console::Logger::getInstance().setCallback(timeLevelCallback);
        cali.setLogger(logger);
        // system_cfg_file = "config/20260622/calibrated/system_4_ND_20260622_4_19.ini";
        SPDLOG_LOGGER_WARN(logger, "Starting calibration...");
        assert(fs::exists(measure_cfg_file) && "The measure configuration file does not exist.");
        assert(fs::exists(system_cfg_file) && "The system configuration file does not exist.");
        SPDLOG_LOGGER_INFO(logger, "Loading configuration from {} and {}", system_cfg_file, measure_cfg_file.path());
        SPDLOG_LOGGER_INFO(logger, "Processing Ring {}", measure_cfg_file.path().stem());
        SPDLOG_LOGGER_INFO(spdlog::default_logger(), "Processing Ring {}", measure_cfg_file.path().stem());
        RingMeasure::SystemCfg sys;
        RingMeasure::MeasureCfg measure;
        RingMeasure::loadConfig(system_cfg_file, measure_cfg_file.path(), sys, measure);
        SPDLOG_LOGGER_INFO(logger, "Loaded configuration from {} and {}", system_cfg_file, measure_cfg_file.path());
        #ifdef RING_MEASURE_VERIFY_ONLY
        assert(fs::exists(cali_path/measure_cfg_file.path().stem()) && "The binary configuration file does not exist.");
        std::ifstream in_file(cali_path/measure_cfg_file.path().stem(), std::ios::binary);
        std::array<RingMeasure::ScanCfg, static_cast<size_t>(rm::ScanLocation::NUM)> &
            cfgs_read = sys.robot.scans;
        in_file.read(
                reinterpret_cast<char*>(cfgs_read.data()),
                cfgs_read.size() * sizeof(RingMeasure::ScanCfg)
                );
        in_file.close();
        #endif

        cali.setMeasure(measure);
        cali.setSystem(sys);
    
        // Specify the path to the file system.
        fs::path root_out_path = "result";
        fs::path path = root_out_path / (measure.ring.type + "_" + measure_cfg_file.path().stem().string());
        CREATE_DIR(logger, path);

        #ifndef RING_MEASURE_VERIFY_ONLY
            cali.setPhaseType(rm::PhaseType::CALI);
            cali.calibration(rm::ScanLocation::Out, path);
        #endif

        #ifndef RING_MEASURE_CALI_ONLY
            cali.setPhaseType(rm::PhaseType::VERI);
            cali.verification(rm::ScanLocation::Out, path);
        #endif

        #ifndef RING_MEASURE_VERIFY_ONLY
            cali.setPhaseType(rm::PhaseType::CALI);
            cali.calibration(rm::ScanLocation::In, path);
        #endif

        #ifndef RING_MEASURE_CALI_ONLY
            cali.setPhaseType(rm::PhaseType::VERI);
            cali.verification(rm::ScanLocation::In, path);
        #endif

        #ifndef RING_MEASURE_VERIFY_ONLY
            cali.setPhaseType(rm::PhaseType::CALI);
            cali.calibration(rm::ScanLocation::Under, path);
        #endif

        #ifndef RING_MEASURE_CALI_ONLY
            cali.setPhaseType(rm::PhaseType::VERI);
            cali.verification(rm::ScanLocation::Under, path);
        #endif

        // dumpConfig("stdout", cali.getSystem());

        visualizeObliquity(cali,  path / "obliquity.ply");

        // Dump the new configuratrion to a ini file.
        // Do not dump the configuration in verifying only mode.
        #ifndef RING_MEASURE_VERIFY_ONLY
            fs::path new_ini = "system_" + measure_cfg_file.path().filename().string();
            SPDLOG_LOGGER_INFO(logger, "Dumping the new configuration to {}", cali_path/new_ini);
            auto sys_cali = cali.getSystem();
            RingMeasure::dumpConfig(cali_path/new_ini, sys_cali);
            // RingMeasure::dumpConfig(cali_path/new_ini, sys);
            auto calibrated = cali.getSystem().robot.scans;
            // std::vector<RingMeasure::ScanCfg> cali_cfgs = {sys.robot.in, sys.robot.out, sys.robot.under};
            // std::vector<RingMeasure::ScanCfg> cali_cfgs = {calibrated.robot.scans[static_cast<size_t>(rm::ScanLocation::In)],
            //                                                calibrated.robot.out,
            //                                                calibrated.robot.under};
            fs::path filename = measure_cfg_file.path().stem().string();
            // Open file in output + binary mode
            SPDLOG_LOGGER_INFO(logger, "Dumping the new binary configuration to {}", cali_path/filename);
            std::ofstream out_file(cali_path/filename, std::ios::binary);
            assert(out_file.is_open() && "Failed to open file for writing!");
            out_file.write(
                reinterpret_cast<const char*>(calibrated.data()),
                calibrated.size() * sizeof(RingMeasure::ScanCfg)
            );
            out_file.close();
        #endif
        logger->flush();

        // #ifndef RING_MEASURE_CALI_ONLY
        // // write the reconstructed point clouds to PLY files for visualization
        // *rec_out += *rec_in;
        // *rec_out += *rec_under;
        // // Downsample the point cloud using a voxel grid filter
        // pcl::VoxelGrid<pcl::PointNormal> vox;
        // vox.setInputCloud(rec_out);
        // int camera_resolution = measure.ring.dA * M_PI / 360.0f;
        // float resample = camera_resolution * 0.75f;
        // vox.setLeafSize(resample, resample, resample); // Set the voxel size (adjust as needed)
        // pcl::PointCloud<pcl::PointNormal>::Ptr out (new pcl::PointCloud<pcl::PointNormal>);
        // vox.filter(*out);
        // SPDLOG_LOGGER_INFO(logger, "Downsampled reconstruction has {} points.", out->size());
        // SPDLOG_LOGGER_INFO(logger, "Writing the reconstruction to {}", path / "reconstructed.ply");
        // SPDLOG_EXEC(logger, spdlog::level::info, pcl::io::savePLYFileASCII(path / "reconstructed.ply", *out));

        // pcl::RegionGrowing<pcl::PointNormal, pcl::PointNormal> reg;
        // pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
        // reg.setResidualThreshold (camera_resolution * 0.3f);
        // reg.setMinClusterSize (1);
        // reg.setMaxClusterSize (out->size());
        // tree->setInputCloud (out);
        // reg.setSearchMethod (tree);
        // reg.setNumberOfNeighbours (8);
        // reg.setInputCloud (out);
        // reg.setInputNormals (out);
        // reg.setSmoothnessThreshold (15.0 / 180.0 * M_PI);
        // reg.setCurvatureThreshold (0.05); // in
        // std::vector <pcl::PointIndices> segmentation_indices;
        // reg.extract (segmentation_indices);
        // std::sort(segmentation_indices.begin(), segmentation_indices.end(),
        //          [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
        //          return a.indices.size() > b.indices.size(); // Sort in descending order of cluster size
        // });
        // SPDLOG_LOGGER_TRACE(logger, "Write the {} clusters of out segmented to: {}", segmentation_indices.size(), path / "recon_seg");
        // SPDLOG_EXEC(logger, spdlog::level::trace, RingMeasure::writeSegmentation<pcl::PointNormal>(out, segmentation_indices, path / "recon_seg", "_RecSeg", 10));
        // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
        // SPDLOG_LOGGER_TRACE(logger, "Saving the colored segmented point cloud to {}", path / "noisy_recon.ply");
        // SPDLOG_EXEC(logger, spdlog::level::trace, pcl::io::savePLYFileASCII(path / "noisy_recon.ply", *colored_cloud));

        // // Extract the segs with large size
        // auto split = std::partition(segmentation_indices.begin(), segmentation_indices.end(),
        //     [&out](const pcl::PointIndices& indices) {
        //         return indices.indices.size() > out->size() * 0.1f; // Adjust the threshold as needed
        //     });
        // int num_of_segs = std::distance(split, segmentation_indices.end());
        // SPDLOG_LOGGER_TRACE(logger, "Number of clusters with size less than 10% of the total points: {}", num_of_segs);
        // cv::Mat feature(num_of_segs, 1, CV_32F); // the feature vector is the size of the clusters
        // int k = 3; // number of clusters in k-means
        // int idx = 0;
        // for (auto it = split; it != segmentation_indices.end(); ++it, ++idx) {
        //     feature.at<float>(idx, 0) = it->indices.size();
        // }
        // cv::Mat labels, centers;
        // cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 1000, 1.0);
        // cv::kmeans(feature, k, labels, criteria, 3, cv::KMEANS_PP_CENTERS, centers);
        // cv::Point min_loc;
        // cv::minMaxLoc(centers, nullptr, nullptr, &min_loc, nullptr);
        // int discard = min_loc.x;
        // SPDLOG_LOGGER_TRACE(logger, "Discarding the cluster with label: {}", discard);
        // pcl::PointIndices::Ptr discard_indices(new pcl::PointIndices); ;
        // int offset = split - segmentation_indices.begin();
        // for (int i = offset; i < num_of_segs; ++i){
        //     SPDLOG_LOGGER_TRACE(logger, "Cluster {:2d} has size {:4d} and is assigned to cluster {}", i, segmentation_indices[i].indices.size(), labels.at<int>(i - offset, 0));
        //     if (labels.at<int>(i - offset, 0) == discard)
        //     {
        //         discard_indices->indices.insert(discard_indices->indices.end(), segmentation_indices[i].indices.begin(), segmentation_indices[i].indices.end());
        //     }
        // }
        // // Extract the lateral and the base clusters
        // pcl::PointCloud<pcl::PointNormal>::Ptr rec(new pcl::PointCloud<pcl::PointNormal>);
        // pcl::ExtractIndices<pcl::PointNormal> extract;
        // extract.setInputCloud(out);
        // extract.setIndices(discard_indices);
        // extract.setNegative(true);
        // extract.filter(*rec);
        // SPDLOG_LOGGER_INFO(logger, "Writing the final reconstruction to {}", path / "recon_pc.ply");
        // SPDLOG_EXEC(logger, spdlog::level::info, pcl::io::savePLYFileASCII(path / "recon_pc.ply", *rec));

        // pcl::search::KdTree<pcl::PointNormal>::Ptr gp3_tree(new pcl::search::KdTree<pcl::PointNormal>());
        // gp3_tree->setInputCloud(rec);
    
        // // Initialize GP3 reconstructor
        // pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        // pcl::PolygonMesh mesh;
    
        // // Set search radius (max distance between connected vertices)
        // // Keep this slightly larger than your point spacing, but small enough to preserve holes
        // gp3.setSearchRadius(resample*2.0);
        // // Set triangulation parameters
        // gp3.setMu(2.5);                         // Multiplier for nearest-neighbor distance
        // gp3.setMaximumNearestNeighbors(100);    // Max neighbors considered per point
        // gp3.setMaximumSurfaceAngle(M_PI / 4);   // 45 deg max angle between normals
        // gp3.setMinimumAngle(M_PI / 18);         // 10 deg min triangle angle
        // gp3.setMaximumAngle(2 * M_PI / 3);      // 120 deg max triangle angle
        // gp3.setNormalConsistency(false);        // Ensure consistent face normals
    
        // // Connect input cloud and search tree to GP3
        // gp3.setInputCloud(rec);
        // gp3.setSearchMethod(gp3_tree);
    
        // // Reconstruct mesh
        // gp3.reconstruct(mesh);

        // SPDLOG_LOGGER_INFO(logger, "Writing the reconstructed mesh to {}", path / "recon_mesh.ply");
        // SPDLOG_EXEC(logger, spdlog::level::info, pcl::io::savePLYFile(path / "recon_mesh.ply", mesh));
        // #endif

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        SPDLOG_LOGGER_WARN(logger, "Total execution time: {} ms", duration.count());
        SPDLOG_LOGGER_INFO(logger, "Finished processing the scan from {}.\n\n\n", measure_cfg_file.path().stem());
    }
    if(is_file)
    {
        SPDLOG_LOGGER_INFO(spdlog::default_logger(), "Finished processing the measure configuration file: {}", path_arg);
        config_path = path_arg.parent_path();
        single_cfg_file = path_arg.filename();
    }
    else
    {
        SPDLOG_LOGGER_INFO(spdlog::default_logger(), "Finished processing the measure configuration folder: {}", path_arg);
        config_path = path_arg;
    }
    // flush all the logs and terminate the threads safely.
    spdlog::shutdown();
    return (0);
}