// IDetector.h
#pragma once
#include <memory>

class IDetector {
public:
    virtual ~IDetector() = default;

    /// @brief 启动后台检测线程（从全局帧源读取并处理）
    virtual void startDetectionLoop(int width, int height) = 0;

    /// @brief 停止检测线程并等待退出
    virtual void stop() = 0;

    /// @brief 检查是否初始化成功
    virtual bool isInitialized() const = 0;

    /// @brief 检查检测线程是否在运行
    virtual bool isRunning() const = 0;
};