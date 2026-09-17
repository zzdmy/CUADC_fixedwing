#pragma once
#ifndef APP_LOGGER_H
#define APP_LOGGER_H

#include <memory>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

class AppLogger {
public:
    static AppLogger& get();

    template<typename... Args>
    void info(fmt::format_string<Args...> fmt, Args&&... args) {
        logger_->info(fmt, std::forward<Args>(args)...);
    }

    template<typename... Args>
    void warn(fmt::format_string<Args...> fmt, Args&&... args) {
        logger_->warn(fmt, std::forward<Args>(args)...);
    }

    template<typename... Args>
    void error(fmt::format_string<Args...> fmt, Args&&... args) {
        logger_->error(fmt, std::forward<Args>(args)...);
    }

    template<typename... Args>
    void debug(fmt::format_string<Args...> fmt, Args&&... args) {
        logger_->debug(fmt, std::forward<Args>(args)...);
    }

    void setLevel(spdlog::level::level_enum level);

private:
    AppLogger();
    ~AppLogger() = default;
    AppLogger(const AppLogger&) = delete;
    AppLogger& operator=(const AppLogger&) = delete;

    std::shared_ptr<spdlog::logger> logger_;
};

#endif // APP_LOGGER_H
