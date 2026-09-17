<#
.SYNOPSIS
    一键准备 PP-OCRv5 编号识别模型（下载 + 校验 + 字典生成）。

.DESCRIPTION
    1. 从 hf-mirror 镜像下载 PP-OCRv5_mobile_rec 的 inference.onnx 与 inference.yml
    2. 从 inference.yml 提取 character_dict，生成 ppocr_keys_v5.txt（每行一个字符）
    3. 打印数字字符的 CTC 索引，便于和代码里的预期对照
    4. 把 inference.onnx 复制成 config.yaml 期望的 PP-OCRv5_mobile_rec.onnx

    下载采用多策略回退（TLS1.2 / curl.exe / .NET HttpClient），
    解决 PowerShell 走代理时报 "unexpected EOF or 0 bytes from the transport stream" 的问题。

.NOTES
    .engine 不需要手动准备：程序首次启动会用 nvonnxparser 自动构建并缓存。

.EXAMPLE
    pwsh -File .\setup_ocr_model.ps1 -Proxy http://127.0.0.1:7897
    pwsh -File .\setup_ocr_model.ps1                      # 已有系统代理或直连时
#>
[CmdletBinding()]
param(
    [string]$Proxy = "",
    [string]$Repo = "PaddlePaddle/PP-OCRv5_mobile_rec_onnx",
    [string]$Mirror = "https://hf-mirror.com",
    [string]$OutDir = $PSScriptRoot,
    [switch]$Force
)

$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'   # 大幅加速 Invoke-WebRequest

function Write-Step([string]$msg) {
    Write-Host ""
    Write-Host "=== $msg ===" -ForegroundColor Cyan
}
function Write-Ok([string]$msg)   { Write-Host "  $msg" -ForegroundColor Green }
function Write-Warn2([string]$msg){ Write-Host "  $msg" -ForegroundColor Yellow }
function Write-Err2([string]$msg) { Write-Host "  $msg" -ForegroundColor Red }

# ---------------------------------------------------------------- 下载核心
function Invoke-Download {
    param(
        [string]$Url,
        [string]$Dest,
        [string]$ProxyUrl = ""
    )

    # --- 策略 0: 已存在且非空则跳过 ---
    if ((Test-Path $Dest) -and -not $Force) {
        $len0 = (Get-Item $Dest).Length
        if ($len0 -gt 0) {
            Write-Host ("  已存在，跳过 ({0:N1} KB): {1}" -f ($len0/1KB), (Split-Path $Dest -Leaf)) -ForegroundColor DarkGray
            return
        }
    }

    Write-Host "  下载: $Url"
    if ($ProxyUrl) { Write-Host "  代理: $ProxyUrl" -ForegroundColor DarkGray }

    # --- 策略 1: .NET HttpClient（对 TLS/HTTP2 兼容性最好）---
    $ok = $false
    try {
        Add-Type -AssemblyName System.Net.Http -ErrorAction SilentlyContinue
        $handler = New-Object System.Net.Http.HttpClientHandler
        $handler.AllowAutoRedirect = $true
        $handler.AutomaticDecompression = [System.Net.DecompressionMethods]::GZip -bor [System.Net.DecompressionMethods]::Deflate
        if ($ProxyUrl) {
            $handler.Proxy = New-Object System.Net.WebProxy($ProxyUrl, $true)
            $handler.UseProxy = $true
        }
        $client = New-Object System.Net.Http.HttpClient($handler)
        $client.Timeout = [TimeSpan]::FromMinutes(30)
        $client.DefaultRequestHeaders.UserAgent.ParseAdd("Mozilla/5.0 setup-ocr")

        $resp = $client.GetAsync($Url, [System.Net.Http.HttpCompletionOption]::ResponseHeadersRead).GetAwaiter().GetResult()
        $resp.EnsureSuccessStatusCode() | Out-Null
        $inStream = $resp.Content.ReadAsStreamAsync().GetAwaiter().GetResult()
        $outStream = [System.IO.File]::Create($Dest)
        try { $inStream.CopyTo($outStream) } finally { $outStream.Dispose(); $inStream.Dispose(); $client.Dispose() }
        $ok = (Test-Path $Dest) -and ((Get-Item $Dest).Length -gt 0)
        if ($ok) { Write-Ok ("HttpClient 下载完成 ({0:N1} KB)" -f ((Get-Item $Dest).Length/1KB)) }
    }
    catch {
        Write-Warn2 "策略 HttpClient 失败: $($_.Exception.Message)"
        if (Test-Path $Dest) { Remove-Item $Dest -Force -ErrorAction SilentlyContinue }
    }

    # --- 策略 2: curl.exe（Windows 10+ 自带，TLS 栈独立）---
    if (-not $ok) {
        $curl = Get-Command curl.exe -ErrorAction SilentlyContinue
        if ($curl) {
            try {
                $curlArgs = @('-L', '--fail', '--silent', '--show-error', '--retry', '3', '--retry-delay', '2',
                              '--connect-timeout', '30', '-o', $Dest)
                if ($ProxyUrl) { $curlArgs += @('-x', $ProxyUrl) }
                $curlArgs += $Url
                & $curl.Source @curlArgs
                $ok = ($LASTEXITCODE -eq 0) -and (Test-Path $Dest) -and ((Get-Item $Dest).Length -gt 0)
                if ($ok) { Write-Ok ("curl 下载完成 ({0:N1} KB)" -f ((Get-Item $Dest).Length/1KB)) }
                else { Write-Warn2 "策略 curl.exe 失败 (exit=$LASTEXITCODE)" }
            }
            catch {
                Write-Warn2 "策略 curl.exe 异常: $($_.Exception.Message)"
            }
            if (-not $ok -and (Test-Path $Dest)) { Remove-Item $Dest -Force -ErrorAction SilentlyContinue }
        }
        else {
            Write-Warn2 "未找到 curl.exe，跳过该策略"
        }
    }

    # --- 策略 3: Invoke-WebRequest ---
    if (-not $ok) {
        try {
            [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
            $iwrArgs = @{
                Uri             = $Url
                OutFile         = $Dest
                UseBasicParsing = $true
                TimeoutSec      = 1800
                UserAgent       = 'Mozilla/5.0 setup-ocr'
            }
            if ($ProxyUrl) { $iwrArgs['Proxy'] = $ProxyUrl }
            Invoke-WebRequest @iwrArgs
            $ok = (Test-Path $Dest) -and ((Get-Item $Dest).Length -gt 0)
            if ($ok) { Write-Ok ("Invoke-WebRequest 下载完成 ({0:N1} KB)" -f ((Get-Item $Dest).Length/1KB)) }
        }
        catch {
            Write-Warn2 "策略 Invoke-WebRequest 失败: $($_.Exception.Message)"
            if (Test-Path $Dest) { Remove-Item $Dest -Force -ErrorAction SilentlyContinue }
        }
    }

    if (-not $ok) {
        throw "所有下载策略均失败: $Url`n  请检查代理是否可用，或用浏览器手动下载后放到 $Dest"
    }
}

# ---------------------------------------------------------------- 内容校验
function Test-YmlContent {
    param([string]$Path)
    $text = [System.IO.File]::ReadAllText($Path)
    if ($text -notmatch 'character_dict:') {
        throw "inference.yml 内容异常：未找到 character_dict（可能是 HTML 错误页）"
    }
    if ($text -notmatch 'model_name') {
        throw "inference.yml 内容异常：未找到 model_name"
    }
    Write-Ok "inference.yml 内容校验通过"
}

function Test-OnnxContent {
    param([string]$Path)
    $fs = [System.IO.File]::OpenRead($Path)
    try {
        $buf = New-Object byte[] 16
        $read = $fs.Read($buf, 0, 16)
        if ($read -lt 8) { throw "ONNX 文件长度不足" }
    }
    finally { $fs.Dispose() }

    # ONNX 是 protobuf：首字节应为 0x08 (field 1, varint = ir_version)
    if ($buf[0] -ne 0x08) {
        $hex = ($buf | ForEach-Object { $_.ToString('X2') }) -join ' '
        throw "ONNX 文件头异常（前 16 字节: $hex），疑似下载到了 HTML 页面。"
    }
    $kb = (Get-Item $Path).Length / 1KB
    if ($kb -lt 1024) {
        Write-Warn2 ("ONNX 只有 {0:N0} KB，官方文件约 16 MB，请确认是否完整" -f $kb)
    }
    Write-Ok ("ONNX 文件头校验通过 ({0:N1} KB)" -f $kb)
}

# ---------------------------------------------------------------- 主流程
Write-Step "准备输出目录"
if (-not (Test-Path $OutDir)) { New-Item -ItemType Directory -Path $OutDir | Out-Null }
Write-Host "  $OutDir"

$ymlUrl  = "$Mirror/$Repo/raw/main/inference.yml"
$onnxUrl = "$Mirror/$Repo/resolve/main/inference.onnx"

$ymlPath  = Join-Path $OutDir 'inference.yml'
$onnxRaw  = Join-Path $OutDir 'inference.onnx'
$onnxPath = Join-Path $OutDir 'PP-OCRv5_mobile_rec.onnx'
$dictPath = Join-Path $OutDir 'ppocr_keys_v5.txt'

Write-Step "下载 PaddleOCR 配置与模型"
Invoke-Download -Url $ymlUrl  -Dest $ymlPath -ProxyUrl $Proxy
Test-YmlContent -Path $ymlPath

Invoke-Download -Url $onnxUrl -Dest $onnxRaw -ProxyUrl $Proxy
Test-OnnxContent -Path $onnxRaw

Write-Step "对齐文件名（config.yaml 期望 PP-OCRv5_mobile_rec.onnx）"
if (Test-Path $onnxPath) {
    $sameSize = (Get-Item $onnxPath).Length -eq (Get-Item $onnxRaw).Length
    if ($sameSize) {
        Write-Host "  已存在同名文件且大小一致，跳过" -ForegroundColor DarkGray
    }
    else {
        Copy-Item $onnxRaw $onnxPath -Force
        Write-Ok "已更新 PP-OCRv5_mobile_rec.onnx"
    }
}
else {
    Copy-Item $onnxRaw $onnxPath -Force
    Write-Ok "已生成 PP-OCRv5_mobile_rec.onnx"
}

Write-Step "从 inference.yml 提取字符字典"
$lines = Get-Content -LiteralPath $ymlPath -Encoding UTF8
$inDict = $false
$chars = New-Object System.Collections.Generic.List[string]

foreach ($line in $lines) {
    if (-not $inDict) {
        if ($line -match '^\s*character_dict:\s*$') { $inDict = $true }
        continue
    }
    if ($line -match '^\s*-\s?(.*)$') {
        # 关键：只能去 ASCII 空白。默认 .Trim() 会把 U+3000 全角空格也删掉，
        # 而字典第 0 项正是全角空格 —— 删掉会导致其后所有字符的 CTC 索引偏移 1 位，
        # 数字识别将整体错位。
        $val = $Matches[1].Trim([char]0x20, [char]0x09, [char]0x0D, [char]0x0A)
        if ($val.Length -eq 0) { continue }
        if ($val.Length -ge 2 -and $val[0] -eq "'" -and $val[-1] -eq "'") {
            $val = $val.Substring(1, $val.Length - 2).Replace("''", "'")
        }
        elseif ($val.Length -ge 2 -and $val[0] -eq '"' -and $val[-1] -eq '"') {
            $val = $val.Substring(1, $val.Length - 2)
        }
        $chars.Add($val)
        continue
    }
    if ($line.Trim([char]0x20, [char]0x09).Length -gt 0) { break }
}

if ($chars.Count -lt 1000) { throw "字典解析异常，只拿到 $($chars.Count) 项（预期约 16500）" }
Write-Ok "解析到 $($chars.Count) 个字符"

# 注意：第 0 项是「全角空格」，是合法字符，写文件时不能丢
$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllLines($dictPath, [string[]]$chars, $utf8NoBom)
Write-Ok "已写出: $dictPath"

Write-Step "数字字符索引自检 (CTC 索引 = 字典下标 + 1)"
$half = @()
for ($i = 0; $i -lt $chars.Count; $i++) {
    if ($chars[$i] -match '^[0-9]$') { $half += ("'{0}'->{1}" -f $chars[$i], ($i + 1)) }
}
$full = @()
for ($i = 0; $i -lt $chars.Count; $i++) {
    if ($chars[$i] -match '^[\uFF10-\uFF19]$') { $full += ("'{0}'->{1}" -f $chars[$i], ($i + 1)) }
}
Write-Host ("  半角数字: " + ($half -join ', '))
if ($full.Count -gt 0) { Write-Host ("  全角数字: " + ($full -join ', ')) }
Write-Host ("  数字候选总数: {0} (半角 {1} + 全角 {2})" -f ($half.Count + $full.Count), $half.Count, $full.Count)

Write-Step "全部就绪"
Write-Host @"
生成的文件:
  $onnxPath
  $ymlPath
  $dictPath

模型类别数应为: $($chars.Count + 2)  (字典 $($chars.Count) + blank + 空格)
程序启动时会打印 "OCR 输出张量(CTC) ... C=..." 用于核对。

下一步: 直接运行程序，首次会自动构建 TensorRT 引擎并缓存为
  $(Join-Path $OutDir 'PP-OCRv5_mobile_rec.engine')
"@ -ForegroundColor Green
