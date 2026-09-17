// ConfigManager.h
#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include "config_loader.h"
#include <mutex>

class ConfigManager
{
public:
    static ConfigManager& getInstance()
    {
        static ConfigManager instance;
        return instance;
    }

    const AppConfig& getConfig() const { return config_; }
    void setConfig(const AppConfig& cfg) { config_ = cfg; }

    // 禁用拷贝
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

private:
    ConfigManager() = default;
    AppConfig config_;
};

#endif