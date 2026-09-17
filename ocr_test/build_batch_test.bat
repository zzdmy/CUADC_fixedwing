@echo off
call "d:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
cd /d D:\vs26\CUADC_fixedwing
set TRT=C:\Program Files\NVIDIA GPU Computing Toolkit\TensorRT-10.7.0.23
set CUDAROOT=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6
set VCPKG=C:\Users\asus1\vcpkg\installed\x64-windows
set INCS=/I . /I "G:\opencv\opencv\build\install\include" /I "%TRT%\include" /I "%CUDAROOT%\include" /I "%VCPKG%\include"
cl /nologo /std:c++20 /EHsc /utf-8 /O2 %INCS% /Fe:ocr_test\OcrBatchTest.exe /Fo:ocr_test\ ocr_test\OcrBatchTest.cpp ocr\PaddleOCRRec.cpp ocr\OcrCharset.cpp AppLogger.cpp /link /LIBPATH:"G:\opencv\opencv\build\install\x64\vc17\lib" /LIBPATH:"%TRT%\lib" /LIBPATH:"%CUDAROOT%\lib\x64" /LIBPATH:"%VCPKG%\lib" opencv_world4130.lib cudart.lib nvinfer_10.lib nvonnxparser_10.lib nvinfer_plugin_10.lib spdlog.lib fmt.lib
echo BATCH_BUILD_EXIT=%ERRORLEVEL%
