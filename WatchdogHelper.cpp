#include "WatchdogHelper.h"
#include <thread>

WaitResult WatchdogHelper::waitUntil(
    std::function<bool()> condition,
    const std::atomic<bool>& running,
    std::chrono::milliseconds timeout,
    std::chrono::milliseconds poll_interval)
{
    auto deadline = std::chrono::steady_clock::now() + timeout;
    bool use_timeout = timeout.count() > 0;

    while (running.load()) {
        if (condition()) {
            return WaitResult::ConditionMet;
        }
        if (use_timeout && std::chrono::steady_clock::now() >= deadline) {
            return WaitResult::Timeout;
        }
        std::this_thread::sleep_for(poll_interval);
    }
    return WaitResult::Stopped;
}

WaitResult WatchdogHelper::waitForTimeout(
    const std::atomic<bool>& running,
    std::chrono::milliseconds duration,
    std::chrono::milliseconds poll_interval)
{
    auto deadline = std::chrono::steady_clock::now() + duration;

    while (running.load()) {
        if (std::chrono::steady_clock::now() >= deadline) {
            return WaitResult::Timeout;
        }
        std::this_thread::sleep_for(poll_interval);
    }
    return WaitResult::Stopped;
}
