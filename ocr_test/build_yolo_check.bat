@echo off
cd /d "%~dp0.."
call "D:\Program Files\Microsoft Visual Studio\18\Insiders\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
set TRT=C:\Program Files\NVIDIA GPU Computing Toolkit\TensorRT-10.16.1.11
set CUDAROOT=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.8
cl /nologo /std:c++20 /EHsc /utf-8 /O2 /I "%TRT%\include" /I "%CUDAROOT%\include" /Fe:ocr_test\yolo_engine_check.exe /Fo:ocr_test\ ocr_test\yolo_engine_check.cpp /link /LIBPATH:"%TRT%\lib" /LIBPATH:"%CUDAROOT%\lib\x64" nvinfer_10.lib cudart.lib
echo YOLOCHECK_BUILD_EXIT=%ERRORLEVEL%
