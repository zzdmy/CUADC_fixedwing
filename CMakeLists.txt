cmake_minimum_required(VERSION 3.16)
project(CUADC_fixedwing LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---- 依赖 ----
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui videoio dnn calib3d)
find_package(Threads REQUIRED)

# CUDA runtime（JetPack 装在 /usr/local/cuda；用裸查找更稳，不依赖 CUDAToolkit 的 cmake 配置）
find_path(CUDA_INCLUDE_DIR cuda_runtime.h PATHS /usr/local/cuda/include /usr/include)
find_library(CUDA_CUDART_LIB cudart
  PATHS /usr/local/cuda/lib64 /usr/local/cuda/lib /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)

# TensorRT（JetPack 装在 /usr/include|lib/aarch64-linux-gnu）
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATHS /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu /usr/include)
find_library(TENSORRT_NVINFER nvinfer
  PATHS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/lib)
find_library(TENSORRT_NVINFER_PLUGIN nvinfer_plugin
  PATHS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/lib)
find_library(TENSORRT_NVONNXPARSER nvonnxparser
  PATHS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/lib)

# 其它库（用裸名查找，兼容各发行版）
find_library(YAML_CPP_LIB yaml-cpp)
find_library(FMT_LIB fmt)
find_library(SPDLOG_LIB spdlog)
find_library(GEOGRAPHIC_LIB NAMES GeographicLib Geographic)
find_library(BOOST_SYSTEM_LIB boost_system)

# mavlink 头文件根目录（含 common/、mavlink_types.h 的目录）
if(NOT DEFINED MAVLINK_DIR)
  set(MAVLINK_DIR "${CMAKE_SOURCE_DIR}/mavlink" CACHE PATH "mavlink include root")
endif()

# ---- 源文件 ----
set(SOURCES
  main.cpp
  AppLogger.cpp
  CaptureThread.cpp
  config_loader.cpp
  FrameDispatcher.cpp
  MavlinkProtocolHandler.cpp
  MissionScheduler.cpp
  MotionController.cpp
  OpenCVDNNDetector.cpp
  PixelToGPSConverter.cpp
  realtime_pipeline.cpp
  SerialTransport.cpp
  TcpTransport.cpp
  TensorRTDetector.cpp
  UdpTransport.cpp
  WatchdogHelper.cpp
  yolodecet/inference.cpp
  yolodecet/yolo_tracker.cpp
  yolov8_trt_infer.cpp
  ocr/OcrCharset.cpp
  ocr/OcrDigitReader.cpp
  ocr/PaddleOCRRec.cpp
  ocr/TrtEngine.cpp
  ocr/PaddleDet.cpp
  ocr/TextLineOri.cpp
  ocr/OcrPipeline.cpp
  rtk/NtripClient.cpp
  rtk/RtkInjector.cpp
  tracking/GPSTargetClusterer.cpp
  LinuxDeviceEnumerator.cpp
)

add_executable(cuadc_fixedwing ${SOURCES})

target_include_directories(cuadc_fixedwing PRIVATE
  ${CMAKE_SOURCE_DIR}
  ${MAVLINK_DIR}
  ${TENSORRT_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIR}
)

target_link_libraries(cuadc_fixedwing PRIVATE
  ${OpenCV_LIBS}
  Threads::Threads
  ${CUDA_CUDART_LIB}
  ${TENSORRT_NVINFER}
  ${TENSORRT_NVINFER_PLUGIN}
  ${TENSORRT_NVONNXPARSER}
  ${YAML_CPP_LIB}
  ${FMT_LIB}
  ${SPDLOG_LIB}
  ${GEOGRAPHIC_LIB}
  ${BOOST_SYSTEM_LIB}
)

target_compile_options(cuadc_fixedwing PRIVATE -Wall)
