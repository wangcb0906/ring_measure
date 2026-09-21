#include <vector>
#include <cassert>
#include <iomanip> // to set the precision in inicpp
#include <inicpp.h>
#include <fstream>
#include <omp.h>

// To make SPDLOG_LOGGER_TRACE work with fs::path
#include <spdlog/fmt/std.h>

#include <boost/multi_array.hpp>

#include "visualization.hpp"
#include "application.hpp"

int
main (int argc, char* argv[])
{
    namespace fs = std::filesystem;
    namespace rm = RingMeasure;
    // logger configuration
    auto logger=spdlog::default_logger();
    logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%l] [%s:%#] %v");

    argparse::ArgumentParser program("ring_cali_eval", "0.1");
    evaluation_parser(program, argc, argv);

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
    auto single_cali = program.present<std::string>("--calibration");
    fs::path cali_config_path = cali_path / "calibrations";
    std::vector<fs::path> calis = get_ini_files(cali_config_path, single_cali);

    // Process the measurement configurations.
    fs::path eval_path = cali_path.parent_path();
    if(auto aux = program.present<std::string>("--evaluation"))
    {
        eval_path = aux.value();
    }
    auto single_eval = program.present<std::string>("--test-ring");
    fs::path eval_config_path = eval_path / "config";
    std::vector<fs::path> evals = get_ini_files(eval_config_path, single_eval);

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
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        SPDLOG_LOGGER_INFO(logger, "Thread {} has finished processing the evaluation {}.", omp_get_thread_num(), cali_cfg_file);
        SPDLOG_LOGGER_INFO(logger, "Total execution time: {} ms", duration.count());
        logger->flush();
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

    // flush all the logs and terminate the threads safely.
    spdlog::shutdown();
    return (0);
}