@echo off
call "d:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
cd /d D:\vs26\CUADC_fixedwing
cl /nologo /std:c++20 /EHsc /utf-8 /O2 /I . /Fe:ocr_test\OcrCharsetTest.exe /Fo:ocr_test\ ocr_test\OcrCharsetTest.cpp ocr\OcrCharset.cpp
echo CHARSET_BUILD_EXIT=%ERRORLEVEL%
cl /nologo /std:c++20 /EHsc /utf-8 /O2 /I . /I "C:\Users\asus1\vcpkg\installed\x64-windows\include" /Fe:ocr_test\ConfigSmokeTest.exe /Fo:ocr_test\ ocr_test\ConfigSmokeTest.cpp config_loader.cpp AppLogger.cpp /link /LIBPATH:"C:\Users\asus1\vcpkg\installed\x64-windows\lib" yaml-cpp.lib spdlog.lib fmt.lib
echo CONFIG_BUILD_EXIT=%ERRORLEVEL%
