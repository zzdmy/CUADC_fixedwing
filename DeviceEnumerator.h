#pragma once

#include <vector>
#include <string>
#include <optional>

class DeviceEnumerator {
public:
    struct CameraInfo {
        int index;
        std::string devicePath;
        std::string friendlyName;
    };

    virtual ~DeviceEnumerator() = default;

    virtual std::vector<CameraInfo> enumerateCameras() = 0;
    virtual std::optional<int> findCameraByVidPid(const std::string& vid, const std::string& pid) = 0;
    virtual std::optional<std::string> findComPortByVidPid(const std::string& vid, const std::string& pid) = 0;
};