#pragma once

#include <pcl/console/print.h>
#include <spdlog/spdlog.h>
#include <functional>

/**
 * @def SPDLOG_EXEC
 * @brief Conditionally executes a function call based on the active spdlog level.
 * @param logger The spdlog logger instance to evaluate against.
 * @param level The target log level required for execution.
 * @param func_call The expression or function call to execute.
 */
#ifndef SPDLOG_EXEC
#define SPDLOG_EXEC(logger, level, func_call) \
    do { \
        if (level >= SPDLOG_ACTIVE_LEVEL) { \
            func_call; \
        } \
    } while (0)
#endif

/**
 * @def CREATE_DIR
 * @brief Safely creates a directory path using std::filesystem if it does not already exist.
 * @details Logs an info message via spdlog if the directory is successfully created.
 * @param logger The spdlog logger instance used for reporting.
 * @param path The target directory path (std::filesystem::path compatible).
 */
#define CREATE_DIR(logger, path) \
    do { \
        if(!fs::exists(path) && fs::create_directories(path)){ \
            SPDLOG_LOGGER_INFO(logger, "Created directory: {}", (path).string()); \
        } \
    } while(0)
    
/**
 * @brief Creates a PCL console log handler bridged to spdlog.
 * 
 * @param logger Shared pointer to the spdlog logger instance that will receive the logs.
 * @return A lambda function compatible with PCL's console logging mechanism, 
 *         routing PCL severities to their corresponding spdlog levels.
 */
inline auto createPclLogHandler(std::shared_ptr<spdlog::logger> logger) {
    return [logger](const pcl::console::LogRecord& record) {
        switch (record.level) {
        case pcl::console::L_ERROR:
            SPDLOG_LOGGER_ERROR(logger, "{}", record.message);
            break;
        case pcl::console::L_WARN:
            SPDLOG_LOGGER_WARN(logger, "{}", record.message);
            break;
        case pcl::console::L_DEBUG:
            SPDLOG_LOGGER_DEBUG(logger, "{}", record.message);
            break;
        case pcl::console::L_VERBOSE:
            SPDLOG_LOGGER_TRACE(logger, "{}", record.message);
            break;
        default:
            SPDLOG_LOGGER_INFO(logger, "{}", record.message);
            break;
        }
    };
}