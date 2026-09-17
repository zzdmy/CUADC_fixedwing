#pragma once

#include "DeviceEnumerator.h"

#ifdef _WIN32
class WinDeviceEnumerator : public DeviceEnumerator {
public:
    std::vector<CameraInfo> enumerateCameras() override;
    std::optional<int> findCameraByVidPid(const std::string& vid, const std::string& pid) override;
    std::optional<std::string> findComPortByVidPid(const std::string& vid, const std::string& pid) override;

private:
    std::string toLower(const std::string& s);
};
#endif