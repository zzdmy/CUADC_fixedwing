#include "AppLogger.h"
#include <spdlog/sinks/basic_file_sink.h>
#include <filesystem>
#include <ctime>
#include <sstream>
#include <iomanip>

AppLogger::AppLogger() {
    try {
        // Ensure logs directory exists
        std::filesystem::create_directories("logs");

        // Console sink: colored, info level
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_pattern("[%Y-%m-%d %H:%M:%S] [%^%l%$] %v");
        console_sink->set_level(spdlog::level::info);

        // File sink: plain text, debug level, logs/drone_YYYY-MM-DD_HH-MM-SS.log
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm;
        localtime_s(&tm, &t);
        std::ostringstream oss;
        oss << "logs/drone_" << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << ".log";
        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(oss.str(), false);
        file_sink->set_pattern("[%Y-%m-%d %H:%M:%S] [%l] %v");
        file_sink->set_level(spdlog::level::debug);

        // Combine sinks: logger level must be the lowest of all sinks (debug)
        spdlog::sinks_init_list sink_list = { console_sink, file_sink };
        logger_ = std::make_shared<spdlog::logger>("app", sink_list);
        logger_->set_level(spdlog::level::debug);
        logger_->flush_on(spdlog::level::err);

        spdlog::register_logger(logger_);
    } catch (const spdlog::spdlog_ex&) {
        // Logger already exists, retrieve it
        logger_ = spdlog::get("app");
    }
}

AppLogger& AppLogger::get() {
    static AppLogger instance;
    return instance;
}

void AppLogger::setLevel(spdlog::level::level_enum level) {
    if (logger_) {
        logger_->set_level(level);
    }
}
