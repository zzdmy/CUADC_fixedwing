@echo off
REM Build the full OCR pipeline (det -> ori -> rec) end-to-end test.
REM Uses the same libs as the main project (TensorRT 10.16 + CUDA 12.8 + OpenCV 4.13 + vcpkg).
call "D:\Program Files\Microsoft Visual Studio\18\Insiders\VC\Auxiliary\Build\vcvars64.bat" >nul
cd /d "%~dp0.."

set TRT=C:\Program Files\NVIDIA GPU Computing Toolkit\TensorRT-10.16.1.11
set CUDAROOT=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.8
set VCPKG=C:\Users\33083\vcpkg\installed\x64-windows
set OPENCV=F:\opencv-cuda-4.13\install

set INCS=/I . /I "%OPENCV%\include" /I "%TRT%\include" /I "%CUDAROOT%\include" /I "%VCPKG%\include"

cl /nologo /std:c++20 /EHsc /utf-8 /O2 %INCS% /Fe:ocr_test\OcrPipelineTest.exe /Fo:ocr_test\ ^
   ocr_test\OcrPipelineTest.cpp ^
   ocr\PaddleOCRRec.cpp ocr\OcrCharset.cpp ocr\TrtEngine.cpp ^
   ocr\PaddleDet.cpp ocr\TextLineOri.cpp ocr\OcrPipeline.cpp AppLogger.cpp ^
   /link /LIBPATH:"%OPENCV%\x64\vc18\lib" ^
         /LIBPATH:"%TRT%\lib" /LIBPATH:"%CUDAROOT%\lib\x64" /LIBPATH:"%VCPKG%\lib" ^
         opencv_world4130.lib cudart.lib nvinfer_10.lib nvonnxparser_10.lib nvinfer_plugin_10.lib ^
         fmt.lib

echo PIPELINE_TEST_BUILD_EXIT=%ERRORLEVEL%
