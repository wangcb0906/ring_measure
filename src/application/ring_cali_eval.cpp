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
    program.add_argument("input")
           .help("Path to the calibrations.\n");

    program.add_argument("output")
           .help("Path to the output directory.\n");

    program.add_argument("-e", "--evaluation")
           .help(
                 "Path to the rings to evaluate.\n"
                 "The defaut value is set to $input.\n"
                 )
           .nargs(1);

    program.add_argument("-b", "--binary")
           .help("If it is specified, use binary format to apply the correction.\n"
                 "The defaut value is false.\n"
                 )
           .default_value(false)
           .implicit_value(true);

    program.add_argument("-p", "--parameter")
           .help("Path to the parameter file.\n"
                 "If not specified, use the parameters of the same name of the calibration.\n"
                 "If specified, the parameters will be loaded from the file.\n"
                 "The parameters will be dumped to log.\n"
                 );

    program.add_argument("-t", "--test-ring")
           .help(
                 "If specified, only the test ring will be processed.\n"
                 "Otherwise, all rings in the folder will be processed.\n"
                 "The test ring should be specified as a file name, e.g., 'ring_1.ini'.\n"
                 "The test ring should be located in the 'config' folder.\n"
                );

    program.add_argument("-c", "--calibration")
            .help("Specific calibration to evaluate, otherwise,\n"
                  "all calibrations in the folder will be processed.\n");

    program.add_argument("-m", "--minus-wedge")
           .help("If it is specified, the height of the table will be subtracted by the height of the wedge.\n"
                 "Set it true for calibration and false for measurement.\n"
                 "The defaut value is false.\n"
                 )
           .default_value(false)
           .implicit_value(true);

    program.add_argument("-s", "--suffix")
           .help("Add the suffix to the log_file and output file names.\n"
                 "Otherwise, no effect.\n"
                 "Default value is empty string.")
           .nargs(1);

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception& err) {
        SPDLOG_LOGGER_ERROR(logger, "Error parsing arguments: {}", err.what());
        auto aux = std::ostringstream{};
        aux << program;
        SPDLOG_LOGGER_ERROR(logger, "{}", aux.str());
        return 1;
    }

    // fs::path root = program.get<std::string>("input");

    // Process the calibration configurations.
    fs::path cali_path = program.get<std::string>("input");;
    fs::path paramters = cali_path.string() + ".ini";
    if(auto aux = program.present<std::string>("--parameter")) {
        paramters = aux.value();
    }
    rm::ParamTune param;
    rm::loadParam(paramters, param);
    SPDLOG_LOGGER_INFO(logger, "Loaded parameters from {}.", paramters);

    // Process the calibration configurations.
    fs::path cali_config_path = cali_path / "calibrations";
    std::vector<fs::path> calis;
    if (fs::exists(cali_config_path) && fs::is_directory(cali_config_path)) {
        // Populate vector directly using directory iterator range
        SPDLOG_LOGGER_INFO(logger, "Processing the calibration configuration folder: {}", cali_config_path);
        for (auto const& entry : fs::directory_iterator(cali_config_path)) {
            if (entry.path().extension() == ".ini") {
                calis.push_back(entry.path());
            }
        }
        std::sort(calis.begin(), calis.end());
    } else {
        SPDLOG_LOGGER_ERROR(logger, "Directory {} does not exist.", cali_config_path);
        exit(EXIT_FAILURE);
    }

    // Remove other files if single calibration is specified
    auto single_cali = program.present<std::string>("--calibration");
    if(single_cali) {
        calis.erase(
            std::remove_if(calis.begin(), calis.end(),
                [&single_cali](const fs::path& p) {
                    return p.filename() != fs::path(single_cali.value()).filename();
                }
            ),
            calis.end()
        );
    }

    // Process the measurement configurations.
    fs::path eval_path = cali_path.parent_path();
    if(auto aux = program.present<std::string>("--evaluation"))
    {
        eval_path = aux.value();
    }
    fs::path eval_config_path = eval_path / "config";
    std::vector<fs::path> evals;
    if (fs::exists(eval_config_path) && fs::is_directory(eval_config_path)) {
        // Populate vector directly using directory iterator range
        SPDLOG_LOGGER_INFO(logger, "Processing the measure configuration folder: {}", eval_config_path);
        for (auto const& entry : fs::directory_iterator(eval_config_path)) {
            if (entry.path().extension() == ".ini") {
                evals.push_back(entry.path());
            }
        }
        std::sort(evals.begin(), evals.end());
    } else {
        SPDLOG_LOGGER_ERROR(logger, "Directory {} does not exist.", eval_config_path);
        exit(EXIT_FAILURE);
    }

    // Remove other files if single evaluation is specified.
    auto single_eval = program.present<std::string>("--test-ring");
    if (single_eval)
    {
        evals.erase(
            std::remove_if(evals.begin(), evals.end(),
                [&single_eval](const fs::path& p){ 
                    return p.filename() != fs::path(single_eval.value()).filename(); 
                }
            ),
            evals.end()
        );
    }

    auto system_cfg_file = eval_path / "system.ini"; 
    fs::path measure_root(eval_path.string() + "_cali_eval");
    auto out = program.present<std::string>("output");
    if(out) {
        measure_root = eval_path / out.value();
    }
    CREATE_DIR(logger, measure_root);

    // Create a 3D grid where EVERY cell is a dynamic std::vector
    boost::multi_array<rm::RingCalibration::ScanMetricsArray, 3> eval_scan(
        boost::extents[static_cast<size_t>(rm::ScanLocation::NUM)]
        [calis.size()][evals.size()]
    );
    boost::multi_array<rm::RingCalibration::ReconMetricsArray, 2> eval_recon(
        boost::extents[calis.size()][evals.size()]
    );

    rm::SystemCfg sys_;
    rm::loadSystemConfig(system_cfg_file, sys_);
    // This is done in cali.hpp in the constructor.
    if(program.get<bool>("minus-wedge"))
    {
        sys_.robot.table_h -= sys_.robot.wedge_h;
    } 
    SPDLOG_LOGGER_INFO(logger, "Loaded configuration from {}.", system_cfg_file);
    rm::RingCfg ring_profile;

    thread_local std::shared_ptr<spdlog::logger> thread_active_logger = nullptr;
    // Register the PCL callback ONCE globally before the loop
    pcl::console::Logger::getInstance().setCallback([](const pcl::console::LogRecord& record) {
        if (thread_active_logger) {
            // Dynamically wrap the active thread's logger using your existing function
            auto handler = createPclLogHandler(thread_active_logger);
            handler(record); // Process the PCL log record with the correct logger and metadata
        }
    });

    bool binary = program.get<bool>("--binary");
    auto start = std::chrono::high_resolution_clock::now();
    // Add the OpenMP directive here!
    #pragma omp parallel for schedule(dynamic)
    for (size_t cali_idx = 0; cali_idx < calis.size(); ++cali_idx) {
        
        SPDLOG_LOGGER_INFO(logger, "Thread {} is processing calibration {}.", omp_get_thread_num(), calis[cali_idx]);

        auto& cali_cfg_file = calis[cali_idx];
        std::array<RingMeasure::ScanCfg, rm::NUM> cfgs_read;
        if(binary)
        {
            auto binary_ini = cali_cfg_file.replace_extension(".bin");
            std::ifstream in_file(binary_ini, std::ios::binary);
            in_file.read(
                    reinterpret_cast<char*>(cfgs_read.data()),
                    cfgs_read.size() * sizeof(rm::ScanCfg)
                    );
            in_file.close();
            SPDLOG_LOGGER_INFO(logger, "Loaded binary correction from {}.", binary_ini);
        }
          else
        {
            auto ascii_ini = cali_cfg_file.replace_extension(".ini");;
            SPDLOG_LOGGER_INFO(logger, "Loaded ASCII correction from {}.", ascii_ini);
            rm::SystemCfg __sys;
            rm::loadSystemConfig(ascii_ini, __sys);
            cfgs_read = __sys.robot.scans;
        }

        std::string log_name = cali_cfg_file.stem().string() + ".log";
        auto logger=spdlog::basic_logger_mt(log_name, (measure_root / log_name).string(), true);
        logger->set_level(spdlog::level::trace);
        logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%l] [%s:%#] %v");
        // wrapper the PCl logger with time and level information
        // Bind this thread's execution context to this specific logger
        thread_active_logger = logger;
        
        fs::path out_root_path = measure_root / cali_cfg_file.stem();
        CREATE_DIR(logger, out_root_path);
        auto start_time = std::chrono::high_resolution_clock::now();
        // Start to evaluate the calibration results
        for (size_t eval_idx = 0; eval_idx < evals.size(); ++eval_idx)
        {
            SPDLOG_LOGGER_INFO(logger, 
                "Thread {} is processing evaluation {} with config {}.",
                 omp_get_thread_num(), evals[eval_idx], cali_cfg_file
            );
            const auto& measure_cfg_file = evals[eval_idx];
            rm::RingCalibration cali;
            fs::path ring = measure_cfg_file.filename();
            if(auto suffix=program.present<std::string>("--suffix")){
                ring = ring.stem().string() + "_" + suffix.value() + ring.extension().string();
            }
            cali.setLogger(logger);
            cali.setParam(param);
            cali.setSystem(sys_);
            cali.setSystemCorrection(cfgs_read);

            rm::MeasureCfg measure;
            rm::loadMeasureConfig(measure_cfg_file, measure);
            cali.setMeasure(measure);
            ring_profile = measure.ring;

            // Specify the path to the file system.
            fs::path path = out_root_path / ring.stem().string();
            CREATE_DIR(logger, path);
            for (const auto& scan : rm::SCAN_LOCS)
            {
                cali.verification(scan, path, true);
                auto loc = static_cast<size_t>(scan);
                eval_scan[loc][cali_idx][eval_idx] = cali.scanEval();
                if(ring.string().find("_3_01") != std::string::npos)
                {
                    eval_scan[loc][cali_idx][eval_idx][static_cast<size_t>(rm::RingCalibration::ScanMetrics::H_PLANE_FIT)] -= 10;
                }
                if(cali_cfg_file.string().find("_3_01") != std::string::npos)
                {   
                    eval_scan[loc][cali_idx][eval_idx][static_cast<size_t>(rm::RingCalibration::ScanMetrics::H_PLANE_FIT)] += 10;
                }
            }
            eval_recon[cali_idx][eval_idx] = cali.reconEval();
        }
        logger->flush();
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        SPDLOG_LOGGER_INFO(logger, "Thread {} has finished processing the evaluation {}.", omp_get_thread_num(), cali_cfg_file);
        SPDLOG_LOGGER_INFO(logger, "Total execution time: {} ms", duration.count());
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    SPDLOG_LOGGER_INFO(spdlog::default_logger(), "Total execution time for all evaluations: {} ms", total_duration.count());

    std::string path_str = measure_root.string();
    float da = ring_profile.dA/2;
    float di = ring_profile.dI/2;
    float h = ring_profile.h;
    float table_h = sys_.robot.table_h;
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

    SPDLOG_LOGGER_INFO(spdlog::default_logger(), "Finished processing the measure configuration folder: {}", eval_path);

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