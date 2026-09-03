#pragma once

#include <pcl/console/print.h>
#include <spdlog/spdlog.h>
#include <functional>

#ifndef SPDLOG_EXEC
#define SPDLOG_EXEC(logger, level, func_call) \
    do { \
        if (level >= SPDLOG_ACTIVE_LEVEL) { \
            func_call; \
        } \
    } while (0)
#endif

#define CREATE_DIR(logger, path) \
    do { \
        if (fs::create_directories(path)) { \
            SPDLOG_LOGGER_ERROR(logger, "Created directory: {}", (path).string()); \
        } \
    } while(0)

// Function taking a logger parameter and returning a lambda that captures it
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