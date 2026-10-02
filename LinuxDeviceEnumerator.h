#pragma once

#include "DeviceEnumerator.h"

// Linux/Jetson 下的设备枚举（等价于 Windows 版 WinDeviceEnumerator）
// - 相机：/dev/video* + V4L2 VIDIOC_QUERYCAP + sysfs 读 VID/PID
// - 串口：/dev/ttyUSB* / /dev/ttyACM* + sysfs 读 VID/PID
class LinuxDeviceEnumerator : public DeviceEnumerator {
public:
    std::vector<CameraInfo> enumerateCameras() override;
    std::optional<int> findCameraByVidPid(const std::string& vid, const std::string& pid) override;
    std::optional<std::string> findComPortByVidPid(const std::string& vid, const std::string& pid) override;
};
