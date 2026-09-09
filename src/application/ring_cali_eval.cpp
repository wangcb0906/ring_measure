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
#include <fstream>
#include <argparse/argparse.hpp>
#include <fmt/format.h>
#include <omp.h>

int
main (int argc, char* argv[])
{
    namespace fs = std::filesystem;
    namespace rm = RingMeasure;
    // logger configuration
    auto logger=spdlog::default_logger();
    logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%l] [%s:%#] %v");

    argparse::ArgumentParser program("ring_cali_eval", "0.1");
    program.add_argument("cali")
           .help("Folder to calibrate or a single measure configuration file.");

    program.add_argument("-i", "--input")
           .help("Path to the calibration results")
           .default_value(std::string("result"))
           .help("Path to the calibration results. If not specified, the default is 'result'.\n"
                 "Always read the parameters from the file of the same name.\n");

    program.add_argument("-c", "--calibration")
           .help("Specific calibration to evaluate, otherwise, \n"
                 "all calibrations in the folder will be processed.\n");

    program.add_argument("-e", "--evaluation")
           .help("Specific ring to evaluate, otherwise, \n"
                 "all rings in the folder will be processed.\n");

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception& err) {
        SPDLOG_LOGGER_ERROR(logger, "Error parsing arguments: {}", err.what());
        auto aux = std::ostringstream{};
        aux << program;
        SPDLOG_LOGGER_ERROR(logger, "{}", aux.str());
        return 1;
    }

    fs::path root = program.get<std::string>("cali");

    auto cali_result = program.get<std::string>("--input");
    fs::path cali_path;
    fs::path paramters;
    cali_path = root / cali_result;
    paramters = root / (cali_result + ".ini");

    auto single_cali_file = program.present<std::string>("--calibration");
    auto single_eval_file = program.present<std::string>("--evaluation");

    rm::ParamTune param;
    rm::loadParam(paramters, param);
    SPDLOG_LOGGER_INFO(logger, "Loaded calibration parameters from {}.", paramters);
    fs::path config_path = root / "config";

    std::vector<std::string> calis;
    if (fs::exists(config_path) && fs::is_directory(config_path)) {
        // Populate vector directly using directory iterator range
        SPDLOG_LOGGER_INFO(logger, "Processing the measure configuration folder: {}", config_path);
        for (auto const& entry : fs::directory_iterator(config_path)) {
            calis.push_back(entry.path().stem().string());
        }
        std::sort(calis.begin(), calis.end());
    } else {
        SPDLOG_LOGGER_ERROR(logger, "Directory {} does not exist.", config_path);
        exit(EXIT_FAILURE);
    }

    auto system_cfg_file = root / "system.ini";
    fs::path measure_root(cali_path.string() + "_cali_eval");
    CREATE_DIR(logger, measure_root);

    int size = calis.size();
    // Create a 3D grid where EVERY cell is a dynamic std::vector
    boost::multi_array<rm::RingCalibration::ScanMetricsArray, 3> eval_scan(
        boost::extents[static_cast<size_t>(rm::ScanLocation::NUM)]
        [size][size]
    );
    boost::multi_array<rm::RingCalibration::ReconMetricsArray, 2> eval_recon(
        boost::extents[size][calis.size()]
    );

    rm::SystemCfg sys;
    rm::loadSystemConfig(system_cfg_file, sys);
    SPDLOG_LOGGER_INFO(logger, "Loaded configuration from {}.", system_cfg_file);
    rm::MeasureCfg measure;

    thread_local std::shared_ptr<spdlog::logger> thread_active_logger = nullptr;
    // Register the PCL callback ONCE globally before the loop
    pcl::console::Logger::getInstance().setCallback([](const pcl::console::LogRecord& record) {
        if (thread_active_logger) {
            // Dynamically wrap the active thread's logger using your existing function
            auto handler = createPclLogHandler(thread_active_logger);
            handler(record); // Process the PCL log record with the correct logger and metadata
        }
    });
    auto start = std::chrono::high_resolution_clock::now();
    // Add the OpenMP directive here!
    #pragma omp parallel for schedule(dynamic)
    for (size_t cali_idx = 0; cali_idx < size; ++cali_idx) {
        const auto& cali_cfg_file = calis[cali_idx];
        if(single_cali_file && cali_cfg_file != single_cali_file.value()) {
            continue;
        }
        rm::RingCalibration cali;
        cali.setSystem(sys);
        auto logger=spdlog::basic_logger_mt("MeasureCali_" + std::to_string(cali_idx), 
            (measure_root / ("[" + fmt::format("{:02}", cali_idx) + "]=" + cali_cfg_file + ".log")).string(), true);
        logger->set_level(spdlog::level::trace);
        logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%l] [%s:%#] %v");
        // wrapper the PCl logger with time and level information
        // Bind this thread's execution context to this specific logger
        thread_active_logger = logger;
        cali.setLogger(logger);
        cali.setParam(param);
        // Read the bianary calibration configuration file
        fs::path cali_cfg_bin = cali_path / "calibrations" / (cali_cfg_file + ".bin");
        assert(fs::exists(cali_cfg_bin) && "The binary cali config file does not exist.");
        std::ifstream in_file(cali_cfg_bin, std::ios::binary);
        std::array<RingMeasure::ScanCfg, static_cast<size_t>(rm::ScanLocation::NUM)> cfgs_read;
        in_file.read(
                reinterpret_cast<char*>(cfgs_read.data()),
                cfgs_read.size() * sizeof(RingMeasure::ScanCfg)
                );
        in_file.close();
        cali.setSystemCorrection(cfgs_read);

        fs::path root_out_path = measure_root / cali_cfg_file;
        CREATE_DIR(logger, root_out_path);

        auto start_time = std::chrono::high_resolution_clock::now();
        // Start to evaluate the calibration results

        for (size_t eval_idx = 0; eval_idx < size; ++eval_idx)
        {
            const auto& ring = calis[eval_idx];
            if(single_eval_file && ring != single_eval_file.value()) {
                continue;
            }
            fs::path ring_path = root_out_path / ring;
            CREATE_DIR(logger, ring_path);
            SPDLOG_LOGGER_INFO(logger, "Processing Ring {} with config {}", ring,  cali_cfg_file);
            rm::loadMeasureConfig(config_path / (ring + ".ini"), measure);
            cali.setMeasure(measure);
            for (const auto& scan : rm::SCAN_LOCS)
            {
                cali.verification(scan, ring_path, true);
                auto loc = static_cast<size_t>(scan);
                eval_scan[loc][cali_idx][eval_idx] = cali.scanEval();
                if(ring.find("_3_01") != std::string::npos)
                {
                    eval_scan[loc][cali_idx][eval_idx][static_cast<size_t>(rm::RingCalibration::ScanMetrics::H_PLANE_FIT)] -= 10;
                }
                if(cali_cfg_file.find("_3_01") != std::string::npos)
                {   
                    eval_scan[loc][cali_idx][eval_idx][static_cast<size_t>(rm::RingCalibration::ScanMetrics::H_PLANE_FIT)] += 10;
                }
            }
            eval_recon[cali_idx][eval_idx] = cali.reconEval();
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        SPDLOG_LOGGER_INFO(logger, "Thread {} has finished processing the evaluation {}.\n\n\n", omp_get_thread_num(), cali_cfg_file);
        SPDLOG_LOGGER_INFO(logger, "Total execution time: {} ms", duration.count());
        SPDLOG_LOGGER_INFO(spdlog::default_logger(), "Thread {:2d} has finished processing the evaluation {}.", omp_get_thread_num(), cali_cfg_file);
        SPDLOG_LOGGER_INFO(spdlog::default_logger(), "Total execution time: {} ms", duration.count());
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    SPDLOG_LOGGER_INFO(spdlog::default_logger(), "Total execution time for all evaluations: {} ms", total_duration.count());

    std::string path_str = measure_root.string();
    float da = measure.ring.dA/2;
    float di = measure.ring.dI/2;
    float h = measure.ring.h;
    float table_h = sys.robot.table_h - sys.robot.wedge_h;
    float metadata[4] = {da, di, h, table_h};
    // /py eval.plot_eval($da, $di, $h, $table_h)

    // Write the evaluation results to a bianary file to enable release mode.
    // save 30 seconds to iterate.
    std::ofstream scan_out((measure_root / "scan.bin").string(), std::ios::out | std::ios::binary | std::ios::trunc);
    if (!scan_out.is_open()) {
        throw std::runtime_error("Failed to open file for binary writing: ");
    }
    const auto* shape = eval_scan.shape();
    size_t scan_metrics_len = std::tuple_size<rm::RingCalibration::ScanMetricsArray>::value;
    size_t scan_full_shape[4] = { shape[0], shape[1], shape[2], scan_metrics_len };
    scan_out.write(reinterpret_cast<const char*>(scan_full_shape), 4 * sizeof(size_t));
    scan_out.write(reinterpret_cast<const char*>(metadata), sizeof(metadata));
    scan_out.write(
        reinterpret_cast<const char*>(eval_scan.data()), 
        eval_scan.num_elements() * sizeof(rm::RingCalibration::ScanMetricsArray)
    );

    std::ofstream recon_out((measure_root / "recon.bin").string(), std::ios::out | std::ios::binary | std::ios::trunc);
    if (!recon_out.is_open()) {
        throw std::runtime_error("Failed to open file for binary writing ");
    }
    shape = eval_recon.shape();
    size_t rec_metrics_len = std::tuple_size<rm::RingCalibration::ReconMetricsArray>::value;
    size_t recon_full_shape[3] = { shape[0], shape[1], rec_metrics_len };
    recon_out.write(reinterpret_cast<const char*>(recon_full_shape), 3 * sizeof(size_t));
    recon_out.write(reinterpret_cast<const char*>(metadata), sizeof(metadata));
    recon_out.write(
        reinterpret_cast<const char*>(eval_recon.data()), 
        eval_recon.num_elements() * sizeof(rm::RingCalibration::ReconMetricsArray)
    );

    SPDLOG_LOGGER_INFO(spdlog::default_logger(), "Finished processing the measure configuration folder: {}", config_path);

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