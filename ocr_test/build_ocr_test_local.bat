@echo off
cd /d "%~dp0.."
call "D:\Program Files\Microsoft Visual Studio\18\Insiders\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1

set TRT=C:\Program Files\NVIDIA GPU Computing Toolkit\TensorRT-10.16.1.11
set CUDAROOT=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.8
set VCPKG=C:\Users\33083\vcpkg\installed\x64-windows
set OPENCV=F:\opencv-cuda-4.13\install

set INCS=/I . /I "%OPENCV%\include" /I "%TRT%\include" /I "%CUDAROOT%\include" /I "%VCPKG%\include"

cl /nologo /std:c++20 /EHsc /utf-8 /O2 %INCS% /Fe:ocr_test\OcrRecTest.exe /Fo:ocr_test\ ocr_test\OcrRecTest.cpp ocr\PaddleOCRRec.cpp ocr\OcrCharset.cpp AppLogger.cpp /link /LIBPATH:"%OPENCV%\x64\vc18\lib" /LIBPATH:"%TRT%\lib" /LIBPATH:"%CUDAROOT%\lib\x64" /LIBPATH:"%VCPKG%\lib" opencv_world4130.lib cudart.lib nvinfer_10.lib nvonnxparser_10.lib nvinfer_plugin_10.lib fmt.lib

echo OCRTEST_BUILD_EXIT=%ERRORLEVEL%
