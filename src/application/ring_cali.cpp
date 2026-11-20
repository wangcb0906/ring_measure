#include <vector>
#include <cassert>
#include <optional>
#include <iomanip> // to set the precision in inicpp
#include <inicpp.h>
#include <omp.h>

// To make SPDLOG_LOGGER_TRACE work with fs::path
#include <spdlog/fmt/std.h>
 
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

    argparse::ArgumentParser program("ring_cali", "0.1");
    calibration_parser(program, argc, argv);

    fs::path root = program.get<std::string>("cali");
    if (!fs::exists(root)) {
        SPDLOG_LOGGER_ERROR(logger, "Calibration folder {} does not exist.", root);
        return 1;
    }

    auto parameters = program.present<std::string>("--parameter");
    rm::ParamTune param;
    fs::path out_root = root / "result";
    if(program.is_used("--parameter")){
        auto aux = fs::path(parameters.value());
        out_root = root / aux.stem();
        SPDLOG_LOGGER_INFO(logger, "Output root directory set to {} based on parameter file.", out_root);
        aux = root / aux.filename();
        rm::loadParam(aux, param);
        SPDLOG_LOGGER_INFO(logger, "Loaded calibration parameters from {}.", aux);
    }
    else {
        SPDLOG_LOGGER_INFO(logger, "Using default parameters.");
        SPDLOG_LOGGER_INFO(logger, "Output root directory set to {}.", out_root);
    }

    if(program.is_used("--output")){
        out_root = root / program.get<std::string>("--output");
        SPDLOG_LOGGER_INFO(logger, "Output root directory set to {} based on --output argument.", out_root);
    }

    fs::path config_path = root / "config";
    fs::path log_path = out_root / "logs";
    fs::path cali_path = out_root / "calibrations";
    CREATE_DIR(logger, cali_path);
    SPDLOG_LOGGER_INFO(logger, "Output calibration directory set to {}.", cali_path);

    fs::path system_cfg_file = root / "system.ini";
    rm::SystemCfg sys_;
    rm::loadSystemConfig(system_cfg_file, sys_);
    SPDLOG_LOGGER_INFO(logger, "Loaded configuration from {}.", system_cfg_file);

    auto single_cfg_file = program.present<std::string>("--ring-cfg");
    std::vector<fs::path> calis = get_ini_files(config_path, single_cfg_file);

    thread_local std::shared_ptr<spdlog::logger> thread_active_logger = nullptr;
    // Register the PCL callback ONCE globally before the loop
    pcl::console::Logger::getInstance().setCallback([](const pcl::console::LogRecord& record) {
        if (thread_active_logger) {
            // Dynamically wrap the active thread's logger using your existing function
            auto handler = createPclLogHandler(thread_active_logger);
            handler(record); // Process the PCL log record with the correct logger and metadata
        }
    });

    int size = calis.size();
    auto mode = program.get<int>("--mode");
    bool cali_enabled = (mode & 1) != 0;
    bool verify_enabled = (mode & 2) != 0;

    auto start = std::chrono::high_resolution_clock::now();
    #pragma omp parallel for schedule(dynamic)
    for (int cali_idx = 0; cali_idx < size; ++cali_idx) {
        const auto& measure_cfg_file = calis[cali_idx];

        // If the argument is a file, only process that specific file
        if(single_cfg_file && measure_cfg_file.filename() != single_cfg_file) {
            continue;
        }
        rm::RingCalibration cali;
        fs::path ring = measure_cfg_file.filename();
        if(single_cfg_file && !program.get<std::string>("--suffix").empty()){
            ring = ring.stem().string() + "_" + program.get<std::string>("--suffix")
            + ring.extension().string();
        }

        auto log_file = log_path / (ring.stem().string() + ".log");
        auto logger=spdlog::basic_logger_mt(ring.string(), log_file.string(), true);
        logger->set_level(spdlog::level::trace);
        logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%l] [%s:%#] %v");
        // wrapper the PCl logger with time and level information
        thread_active_logger = logger;
        cali.setLogger(logger);

        cali.setParam(param);

        auto start_time = std::chrono::high_resolution_clock::now();
        SPDLOG_LOGGER_WARN(logger, "Starting calibration...");
        SPDLOG_LOGGER_INFO(logger, "Processing Ring {}", measure_cfg_file.stem());
        SPDLOG_LOGGER_INFO(spdlog::default_logger(), "Thread {:2d} is processing Ring {}",
            omp_get_thread_num(), measure_cfg_file.stem());
        spdlog::default_logger()->flush();
        rm::MeasureCfg measure;
        rm::loadMeasureConfig(measure_cfg_file, measure);
        SPDLOG_LOGGER_INFO(logger, "Loaded configuration from {}.", measure_cfg_file);

        if(program.get<bool>("--binary") && !cali_enabled && verify_enabled)
        {
            assert(fs::exists(cali_path/ring.stem()) && "The binary configuration file does not exist.");
            auto binary = cali_path / (ring.stem().string() + ".bin");
            std::ifstream in_file(binary, std::ios::binary);
            std::array<rm::ScanCfg, static_cast<size_t>(rm::ScanLocation::NUM)> &
                cfgs_read = sys_.robot.scans;
            in_file.read(
                    reinterpret_cast<char*>(cfgs_read.data()),
                    cfgs_read.size() * sizeof(rm::ScanCfg)
                    );
            in_file.close();
            SPDLOG_LOGGER_INFO(logger, "Loaded binary configuration from {}.", binary);
        }
        cali.setSystem(sys_);
        cali.setMeasure(measure);
    
        // Specify the path to the file system.
        fs::path path = out_root / (measure.ring.type + "_" + ring.stem().string());
        for(const auto& scan : rm::SCAN_LOCS)
        {
            try{
                if(cali_enabled)
                    cali.calibration(scan, path);
                if(verify_enabled)
                    cali.verification(scan, path);
            }
            catch(...){
                try {
                    throw; // Re-throw the exception to capture it in the catch block below
                } catch (const std::exception& e) {
                    SPDLOG_LOGGER_ERROR(logger, "Error: {}", e.what());
                    cali.getLogger()->flush();
                } 
            }
        }

        SPDLOG_EXEC(logger, spdlog::level::debug, visualizeObliquity(cali,  path / "obliquity.ply"));

        // Dump the new configuratrion to a ini file.
        // Do not dump the configuration in verifying only mode.
        if(cali_enabled)
        {
            fs::path new_ini = cali_path / ring;
            SPDLOG_LOGGER_INFO(logger, "Dumping the new configuration to {}", new_ini);
            auto sys_cali = cali.getSystem();
            rm::dumpConfig(new_ini, sys_cali);
            auto calibrated = sys_cali.robot.scans;
            fs::path binary_ini = cali_path / (ring.stem().string() + ".bin");
            // Open file in output + binary mode
            SPDLOG_LOGGER_INFO(logger, "Dumping the new binary configuration to {}", binary_ini);
            std::ofstream out_file(binary_ini, std::ios::binary);
            assert(out_file.is_open() && "Failed to open file for writing!");
            out_file.write(
                reinterpret_cast<const char*>(calibrated.data()),
                calibrated.size() * sizeof(rm::ScanCfg)
            );
            out_file.close();
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        SPDLOG_LOGGER_INFO(logger, "Total execution time: {} ms", duration.count());
        SPDLOG_LOGGER_INFO(logger, "Finished processing the scan from {}.\n\n\n", measure_cfg_file.stem());

        logger->flush();
        if(single_cfg_file) {
            dumpParam(log_file, cali.getParam(), std::ios_base::app);
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    SPDLOG_LOGGER_INFO(spdlog::default_logger(),
        "Total execution time for all calibrations: {} ms", total_duration.count());

    if(single_cfg_file)
    {
        SPDLOG_LOGGER_INFO(spdlog::default_logger(),
            "Finished processing the measure configuration file: {}", single_cfg_file.value());
    }
    else
    {   
        SPDLOG_LOGGER_INFO(spdlog::default_logger(),
            "Dumping the parameter file for the measure configuration folder: {}", root);
        dumpParam(root / (out_root.filename().string() + ".ini"),
            param, std::ios_base::out | std::ios::trunc);
        SPDLOG_LOGGER_INFO(spdlog::default_logger(),
            "Finished processing the measure configuration folder: {}", root);
    }
    // flush all the logs and terminate the threads safely.
    spdlog::shutdown();
    return (0);
}