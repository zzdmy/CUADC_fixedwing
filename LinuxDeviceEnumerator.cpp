// LinuxDeviceEnumerator.cpp
#include "LinuxDeviceEnumerator.h"

#include <glob.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>

namespace {

std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

std::string trim(const std::string& s) {
    auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

bool readFile(const std::string& path, std::string& out) {
    std::ifstream f(path);
    if (!f.is_open()) return false;
    std::getline(f, out);
    out = trim(out);
    return !out.empty();
}

std::vector<std::string> globDevices(const std::string& pattern) {
    std::vector<std::string> result;
    glob_t g{};
    if (glob(pattern.c_str(), 0, nullptr, &g) == 0) {
        for (size_t i = 0; i < g.gl_pathc; ++i) result.emplace_back(g.gl_pathv[i]);
    }
    globfree(&g);
    std::sort(result.begin(), result.end());
    return result;
}

// 从 /sys/class/<class>/<name>/device 出发，逐级向上查找 idVendor/idProduct（USB 设备）
bool readUsbVidPid(const std::string& sysfsDev, std::string& vid, std::string& pid) {
    std::string cur = sysfsDev + "/device";
    for (int i = 0; i < 6; ++i) {
        if (readFile(cur + "/idVendor", vid) && readFile(cur + "/idProduct", pid))
            return true;
        cur += "/..";
    }
    return false;
}

std::string queryCameraName(const std::string& devPath) {
    int fd = open(devPath.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0) return "";
    v4l2_capability cap{};
    std::string name;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
        name = reinterpret_cast<const char*>(cap.card);
        if (name.empty()) name = reinterpret_cast<const char*>(cap.driver);
    }
    close(fd);
    return name;
}

int indexFromDevPath(const std::string& path) {
    size_t slash = path.rfind('/');
    size_t pos = (slash == std::string::npos) ? 0 : slash + 1;
    // /dev/video12 -> 12
    size_t d = path.find_first_of("0123456789", pos);
    if (d == std::string::npos) return -1;
    try { return std::stoi(path.substr(d)); }
    catch (...) { return -1; }
}

} // namespace

std::vector<DeviceEnumerator::CameraInfo> LinuxDeviceEnumerator::enumerateCameras() {
    std::vector<CameraInfo> result;
    for (const auto& path : globDevices("/dev/video*")) {
        int idx = indexFromDevPath(path);
        if (idx < 0) continue;
        CameraInfo info;
        info.index = idx;
        info.devicePath = path;
        info.friendlyName = queryCameraName(path);
        result.push_back(std::move(info));
    }
    return result;
}

std::optional<int> LinuxDeviceEnumerator::findCameraByVidPid(const std::string& vid, const std::string& pid) {
    const std::string v = toLower(vid), p = toLower(pid);
    for (const auto& path : globDevices("/dev/video*")) {
        int idx = indexFromDevPath(path);
        if (idx < 0) continue;
        std::string devVid, devPid;
        if (!readUsbVidPid("/sys/class/video4linux/video" + std::to_string(idx), devVid, devPid))
            continue;
        if (toLower(devVid) == v && toLower(devPid) == p) return idx;
    }
    return std::nullopt;
}

std::optional<std::string> LinuxDeviceEnumerator::findComPortByVidPid(const std::string& vid, const std::string& pid) {
    const std::string v = toLower(vid), p = toLower(pid);
    auto match = [&](const std::string& name) {
        std::string devVid, devPid;
        return readUsbVidPid("/sys/class/tty/" + name, devVid, devPid) &&
               toLower(devVid) == v && toLower(devPid) == p;
    };
    for (const auto& path : globDevices("/dev/ttyUSB*")) {
        std::string name = path.substr(path.rfind('/') + 1);
        if (match(name)) return path;
    }
    for (const auto& path : globDevices("/dev/ttyACM*")) {
        std::string name = path.substr(path.rfind('/') + 1);
        if (match(name)) return path;
    }
    return std::nullopt;
}
