@echo off
REM ============================================================
REM  OCR 批量识别测试（自动配好 PATH，无需手动拼长命令）
REM
REM  用法:
REM    ocr_test\run_ocr_test.bat                       测试 images\ 目录
REM    ocr_test\run_ocr_test.bat dataset\images        测试指定目录
REM    ocr_test\run_ocr_test.bat dataset\images --full 不裁 ROI，整图送识别
REM ============================================================
setlocal

set ROOT=%~dp0..
pushd "%ROOT%"

set TRT=C:\Program Files\NVIDIA GPU Computing Toolkit\TensorRT-10.7.0.23
set CUDAROOT=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6
set VCPKG=C:\Users\asus1\vcpkg\installed\x64-windows
set OPENCV=G:\opencv\opencv\build\install\x64\vc17

set PATH=%TRT%\lib;%CUDAROOT%\lib;%CUDAROOT%\bin;%OPENCV%\bin;%VCPKG%\bin;%PATH%

if not exist "ocr_test\OcrBatchTest.exe" (
  echo [ERROR] 未找到 ocr_test\OcrBatchTest.exe
  echo         请先执行: ocr_test\build_batch_test.bat
  popd
  exit /b 1
)

REM 自动切换到 UTF-8，避免中文输出在部分控制台下乱码
chcp 65001 >nul 2>&1

set IMGDIR=%~1
if "%IMGDIR%"=="" set IMGDIR=images
set EXTRA=%~2

if not exist "%IMGDIR%" (
  echo [INFO] 目录不存在，正在创建: %IMGDIR%
  mkdir "%IMGDIR%" 2>nul
  echo.
  echo        请把待测图片放进 %IMGDIR% 后重新运行本脚本。
  echo.
  echo        判分方式:
  echo          1^) 文件名带编号，例如 target_42.jpg  自动取最长数字串
  echo          2^) 建 labels.txt，每行: 文件名 TAB 期望数字
  echo          3^) 都不做，则只识别不判分
  popd
  exit /b 2
)

dir /b /s "%IMGDIR%\*.jpg" "%IMGDIR%\*.jpeg" "%IMGDIR%\*.png" "%IMGDIR%\*.bmp" "%IMGDIR%\*.webp" "%IMGDIR%\*.tif" "%IMGDIR%\*.tiff" >nul 2>&1
if errorlevel 1 (
  echo [INFO] %IMGDIR% 中还没有图片
  echo        支持: .jpg .jpeg .png .bmp .webp .tif .tiff
  echo        放好图片后重新运行本脚本即可。
  popd
  exit /b 2
)

set LABELS=
if exist "labels.txt" if not "%IMGDIR%"=="images" set LABELS=labels.txt
if exist "%IMGDIR%\..\labels.txt" set LABELS=%IMGDIR%\..\labels.txt
if not "%LABELS%"=="" echo 使用标注文件: %LABELS%

echo.
echo ================ 开始批量识别 ================
echo 图片目录: %IMGDIR%
echo =============================================
echo.

if "%LABELS%"=="" (
  ocr_test\OcrBatchTest.exe "%IMGDIR%" PP-OCRv5_mobile_rec.onnx PP-OCRv5_mobile_rec.engine ppocr_keys_v5.txt inference.yml %EXTRA%
) else (
  ocr_test\OcrBatchTest.exe "%IMGDIR%" PP-OCRv5_mobile_rec.onnx PP-OCRv5_mobile_rec.engine ppocr_keys_v5.txt inference.yml "%LABELS%" %EXTRA%
)

set RC=%ERRORLEVEL%
echo.
echo [退出码 %RC%]  0=全部正确  1=有错误样本  2=参数或目录问题
popd
exit /b %RC%