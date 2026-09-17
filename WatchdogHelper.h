#pragma once
#ifndef WATCHDOG_HELPER_H
#define WATCHDOG_HELPER_H

#include <atomic>
#include <chrono>
#include <functional>

enum class WaitResult {
    ConditionMet,  // condition returned true
    Timeout,       // timed out without condition being met
    Stopped        // running_ became false
};

class WatchdogHelper {
public:
    // Wait until condition returns true, or timeout elapses, or running_ becomes false.
    // poll_interval controls how often the condition and running_ are checked.
    // Returns the outcome as a WaitResult.
    static WaitResult waitUntil(
        std::function<bool()> condition,
        const std::atomic<bool>& running,
        std::chrono::milliseconds timeout,
        std::chrono::milliseconds poll_interval = std::chrono::milliseconds(100));

    // Wait for a fixed duration, abortable via running_.
    // Always returns either Timeout or Stopped.
    static WaitResult waitForTimeout(
        const std::atomic<bool>& running,
        std::chrono::milliseconds duration,
        std::chrono::milliseconds poll_interval = std::chrono::milliseconds(100));
};

#endif // WATCHDOG_HELPER_H
