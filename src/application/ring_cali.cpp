#include <vector>
#include <cassert>
#include <optional>
#include <iomanip> // to set the precision in inicpp
#include <inicpp.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
// To make SPDLOG_LOGGER_TRACE work with fs::path
#include <spdlog/fmt/std.h>

#include <argparse/argparse.hpp>
 
#include "param.hpp"
#include "visualization.hpp"

#if defined(RING_MEASURE_CALI_ONLY) && defined(RING_MEASURE_VERIFY_ONLY)
    #error "Cannot define both RING_MEASURE_CALI_ONLY and RING_MEASURE_VERIFY_ONLY!"
#endif

int
main (int argc, char* argv[])
{
    namespace fs = std::filesystem;
    namespace rm = RingMeasure;

    // logger configuration
    auto logger=spdlog::default_logger();
    logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%l] [%s:%#] %v");

    argparse::ArgumentParser program("ring_cali", "0.1");
    program.add_argument("cali")
           .help("Folder to calibrate or a single measure configuration file.");

    program.add_argument("-p", "--parameter")
           .help("Path to the parameter file."
                 "If not specified, the default parameters will be used, the out_path will be set to 'result'."
                 "If specified, the parameters will be loaded from the file, and the out_path will be set to folder of the same name."
                 );

    program.add_argument("-o", "--output")
           .help("Path to the output directory."
                 "If not specified, the output will be set to 'result' in the root folder"
                 "or in accordance with the value of 'parameter'."
                 "If specified, the output will be overwritten."
                 );

    program.add_argument("-r", "--ring-cfg")
       .help("Specific ring to calibrate, otherwise, "
             "all rings in the folder will be processed."
             "If -r specified, the parameters will be dumped to the log file."
             "Otherwise, the parameters will be dumped to the parameters.ini file in the root folder.");

    program.add_argument("-s", "--suffix")
           .help("If -r specified, add the suffix to the log_file name."
                 "Otherwise, no effect"
                 "Usually, used to distinguish different runs of different parameters on the same ring.")
           .default_value(std::string(""));

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception& err) {
        SPDLOG_LOGGER_ERROR(logger, "Error parsing arguments: {}", err.what());
        SPDLOG_LOGGER_ERROR(logger, "{}", program.usage());
        return 1;
    }

    fs::path root = program.get<std::string>("cali");

    if (!fs::exists(root)) {
        SPDLOG_LOGGER_ERROR(logger, "Calibration folder {} does not exist.", root);
        return 1;
    }

    rm::RingCalibration cali;
    auto parameters = program.present<std::string>("--parameter");
    rm::ParamTune param;
    fs::path out_root = root / "result";
    if(program.is_used("--parameter")){
        auto aux = fs::path(parameters.value());
        out_root = root / aux.stem();
        SPDLOG_LOGGER_INFO(logger, "Output root directory set to {} based on parameter file.", out_root);
        rm::loadParam(aux, param);
        SPDLOG_LOGGER_INFO(logger, "Loaded calibration parameters from {}.", aux);
        cali.setParam(param);
    }
    
    else {
        out_root = root / "result";
        SPDLOG_LOGGER_INFO(logger, "Using default parameters.");
        SPDLOG_LOGGER_INFO(logger, "Output root directory set to {}.", out_root);
    }

    fs::path config_path = root / "config";
    fs::path log_path = out_root / "logs";
    CREATE_DIR(logger, log_path);
    SPDLOG_LOGGER_INFO(logger, "Output log directory set to {}.", log_path);
    fs::path cali_path = out_root / "calibrations";
    CREATE_DIR(logger, cali_path);
    SPDLOG_LOGGER_INFO(logger, "Output calibration directory set to {}.", cali_path);

    fs::path system_cfg_file = root / "system.ini";
    rm::SystemCfg sys_;
    rm::loadSystemConfig(system_cfg_file, sys_);
    SPDLOG_LOGGER_INFO(logger, "Loaded configuration from {}.", system_cfg_file);

    auto single_cfg_file = program.present<std::string>("--ring-cfg");

    for (const auto& measure_cfg_file : fs::directory_iterator(config_path)) {

        // If the argument is a file, only process that specific file
        if(single_cfg_file && measure_cfg_file.path().filename() != single_cfg_file) {
            continue;
        }

        auto start_time = std::chrono::high_resolution_clock::now();

        fs::path ring = measure_cfg_file.path().filename();
        if(single_cfg_file){
            ring = ring.stem().string() + program.get<std::string>("--suffix") + ring.extension().string();
        }
        auto log_file = log_path / ring.replace_extension(".log");
        logger=spdlog::basic_logger_mt(ring, log_file, true);
        logger->set_level(spdlog::level::trace);
        logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%l] [%s:%#] %v");
        // wrapper the PCl logger with time and level information
        auto timeLevelCallback = createPclLogHandler(logger);
        pcl::console::Logger::getInstance().setCallback(timeLevelCallback);
        cali.setLogger(logger);
        // system_cfg_file = "config/20260622/calibrated/system_4_ND_20260622_4_19.ini";
        SPDLOG_LOGGER_WARN(logger, "Starting calibration...");
        SPDLOG_LOGGER_INFO(logger, "Processing Ring {}", measure_cfg_file.path().stem());
        SPDLOG_LOGGER_INFO(spdlog::default_logger(), "Processing Ring {}", measure_cfg_file.path().stem());
        rm::MeasureCfg measure;
        rm::loadMeasureConfig(measure_cfg_file.path(), measure);
        SPDLOG_LOGGER_INFO(logger, "Loaded configuration from {}.", measure_cfg_file.path());
        #ifdef RING_MEASURE_VERIFY_ONLY
        assert(fs::exists(cali_path/ring.stem()) && "The binary configuration file does not exist.");
        auto binary = cali_path/ring.stem();
        std::ifstream in_file(binary, std::ios::binary);
        std::array<rm::ScanCfg, static_cast<size_t>(rm::ScanLocation::NUM)> &
            cfgs_read = sys_.robot.scans;
        in_file.read(
                reinterpret_cast<char*>(cfgs_read.data()),
                cfgs_read.size() * sizeof(rm::ScanCfg)
                );
        in_file.close();
        SPDLOG_LOGGER_INFO(logger, "Loaded binary configuration from {}.", binary);
        #endif

        cali.setMeasure(measure);
        cali.setSystem(sys_);
    
        // Specify the path to the file system.
        fs::path path = out_root / (measure.ring.type + "_" + ring.stem().string());
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
            fs::path new_ini = cali_path / ("system_" + ring.string());
            SPDLOG_LOGGER_INFO(logger, "Dumping the new configuration to {}", new_ini);
            auto sys_cali = cali.getSystem();
            rm::dumpConfig(new_ini, sys_cali);
            auto calibrated = cali.getSystem().robot.scans;
            fs::path binary_ini = cali_path / ring.stem();
            // Open file in output + binary mode
            SPDLOG_LOGGER_INFO(logger, "Dumping the new binary configuration to {}", binary_ini);
            std::ofstream out_file(binary_ini, std::ios::binary);
            assert(out_file.is_open() && "Failed to open file for writing!");
            out_file.write(
                reinterpret_cast<const char*>(calibrated.data()),
                calibrated.size() * sizeof(rm::ScanCfg)
            );
            out_file.close();
        #endif

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
        // SPDLOG_EXEC(logger, spdlog::level::trace, rm::writeSegmentation<pcl::PointNormal>(out, segmentation_indices, path / "recon_seg", "_RecSeg", 10));
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

        logger->flush();
        if(single_cfg_file) {
            dumpParam(log_file, cali.getParam(), std::ios_base::app);
        }
    }

    // TODO: 
    // dump the parameter file.

    if(single_cfg_file)
    {
        SPDLOG_LOGGER_INFO(spdlog::default_logger(), "Finished processing the measure configuration file: {}", single_cfg_file.value());
    }
    else
    {   
        SPDLOG_LOGGER_INFO(spdlog::default_logger(), "Dumping the parameter file for the measure configuration folder: {}", root);
        dumpParam(root / "parameters.ini", cali.getParam(), std::ios_base::app);
        SPDLOG_LOGGER_INFO(spdlog::default_logger(), "Finished processing the measure configuration folder: {}", root);
    }
    // flush all the logs and terminate the threads safely.
    spdlog::shutdown();
    return (0);
}