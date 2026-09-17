#include "WinDeviceEnumerator.h"
#ifdef _WIN32

#include <windows.h>
#include <dshow.h>
#include <initguid.h>
#include <comdef.h>
#include <SetupAPI.h>
#include <devguid.h>
#include <regstr.h>
#include <algorithm>
#include "AppLogger.h"
#include <vector>

#pragma comment(lib, "strmiids.lib")
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "oleaut32.lib")
#pragma comment(lib, "setupapi.lib")

class ComInitializer {
public:
    ComInitializer() {
        HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        if (FAILED(hr)) {
            throw std::runtime_error("COM 初始化失败");
        }
    }
    ~ComInitializer() { CoUninitialize(); }
    ComInitializer(const ComInitializer&) = delete;
    ComInitializer& operator=(const ComInitializer&) = delete;
};

std::string WinDeviceEnumerator::toLower(const std::string& s) {
    std::string lower = s;
    std::transform(lower.begin(), lower.end(), lower.begin(),
        [](unsigned char c) { return std::tolower(c); });
    return lower;
}

std::vector<DeviceEnumerator::CameraInfo> WinDeviceEnumerator::enumerateCameras() {
    ComInitializer comInit;
    ICreateDevEnum* pDevEnum = nullptr;
    IEnumMoniker* pEnum = nullptr;
    std::vector<CameraInfo> cameras;

    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, nullptr,
        CLSCTX_INPROC_SERVER, IID_ICreateDevEnum,
        reinterpret_cast<void**>(&pDevEnum));
    if (FAILED(hr)) return cameras;

    hr = pDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEnum, 0);
    if (FAILED(hr) || pEnum == nullptr) {
        pDevEnum->Release();
        return cameras;
    }

    pEnum->Reset();
    IMoniker* pMoniker = nullptr;
    int index = 0;

    while (pEnum->Next(1, &pMoniker, nullptr) == S_OK) {
        CameraInfo info{ index, "", "" };
        IPropertyBag* pPropBag = nullptr;
        hr = pMoniker->BindToStorage(0, 0, IID_IPropertyBag, (void**)(&pPropBag));
        if (SUCCEEDED(hr)) {
            VARIANT var;
            VariantInit(&var);

            // DevicePath
            if (SUCCEEDED(pPropBag->Read(L"DevicePath", &var, 0)) && var.vt == VT_BSTR) {
                std::wstring wpath(var.bstrVal);
                info.devicePath = std::string(wpath.begin(), wpath.end());
                VariantClear(&var);
            }

            // FriendlyName
            VariantInit(&var);
            if (SUCCEEDED(pPropBag->Read(L"FriendlyName", &var, 0)) && var.vt == VT_BSTR) {
                std::wstring wname(var.bstrVal);
                info.friendlyName = std::string(wname.begin(), wname.end());
                VariantClear(&var);
            }

            pPropBag->Release();
        }
        cameras.push_back(info);
        pMoniker->Release();
        index++;
    }

    pEnum->Release();
    pDevEnum->Release();
    return cameras;
}

std::optional<int> WinDeviceEnumerator::findCameraByVidPid(const std::string& targetVid, const std::string& targetPid) {
    auto cameras = enumerateCameras();
    std::string vidNeedle = toLower("VID_" + targetVid);
    std::string pidNeedle = toLower("PID_" + targetPid);

    for (const auto& cam : cameras) {
        std::string pathLower = toLower(cam.devicePath);
        if (pathLower.find(vidNeedle) != std::string::npos &&
            pathLower.find(pidNeedle) != std::string::npos) {
            AppLogger::get().info(">>> 找到摄像头: VID_{}, PID_{} (索引: {}) <<<", targetVid, targetPid, cam.index);
            return cam.index;
        }
    }
    return std::nullopt;
}

std::optional<std::string> WinDeviceEnumerator::findComPortByVidPid(const std::string& targetVid, const std::string& targetPid) {
    HDEVINFO hDevInfo = SetupDiGetClassDevs(&GUID_DEVINTERFACE_COMPORT, nullptr, nullptr,
        DIGCF_DEVICEINTERFACE | DIGCF_PRESENT);
    if (hDevInfo == INVALID_HANDLE_VALUE) {
        return std::nullopt;
    }

    SP_DEVINFO_DATA devInfo{};
    devInfo.cbSize = sizeof(devInfo);
    DWORD index = 0;
    std::string target = "VID_" + targetVid + "&PID_" + targetPid;
    std::string targetLower = toLower(target);

    while (SetupDiEnumDeviceInfo(hDevInfo, index++, &devInfo)) {
        HKEY hKey = SetupDiOpenDevRegKey(hDevInfo, &devInfo, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
        if (hKey == INVALID_HANDLE_VALUE) continue;

        char portName[256] = { 0 };
        DWORD size = sizeof(portName);
        if (RegQueryValueExA(hKey, "PortName", nullptr, nullptr, (LPBYTE)portName, &size) != ERROR_SUCCESS) {
            RegCloseKey(hKey);
            continue;
        }

        DWORD hwIdSize = 0;
        SetupDiGetDeviceRegistryPropertyA(hDevInfo, &devInfo, SPDRP_HARDWAREID, nullptr, nullptr, 0, &hwIdSize);
        if (hwIdSize > 0) {
            std::vector<char> buffer(hwIdSize);
            if (SetupDiGetDeviceRegistryPropertyA(hDevInfo, &devInfo, SPDRP_HARDWAREID, nullptr,
                (PBYTE)buffer.data(), hwIdSize, nullptr)) {

                const char* p = buffer.data();
                while (*p) {
                    std::string hwId(p);
                    if (toLower(hwId).find(targetLower) != std::string::npos) {
                        AppLogger::get().info("找到 COM 端口: {} (对应 {})", portName, target);
                        RegCloseKey(hKey);
                        SetupDiDestroyDeviceInfoList(hDevInfo);
                        return std::string(portName);
                    }
                    p += strlen(p) + 1;
                }
            }
        }
        RegCloseKey(hKey);
    }

    SetupDiDestroyDeviceInfoList(hDevInfo);
    return std::nullopt;
}

#endif // _WIN32