# 烧录固件到 STM32（像 CCS 的烧录按钮）
#   .\tools\flash.ps1          只烧录
#   .\tools\flash.ps1 -Build   先编译再烧录
# ★ 使用 DAPLink / CMSIS-DAP 无线调试器 + pyocd（不再依赖 ST-Link）
param([switch]$Build)

$root = Split-Path -Parent $PSScriptRoot
$hex  = Join-Path $root 'build\XXU-RoboMaster-Sentry.hex'
$target = 'stm32f407ig'

Write-Host ''
Write-Host '===== XXU-RoboMaster-Sentry =====' -ForegroundColor Cyan

if (-not (Get-Command pyocd -ErrorAction SilentlyContinue)) {
    Write-Host '[X] 找不到 pyocd' -ForegroundColor Red
    Write-Host '    安装： pip install pyocd'
    Write-Host '    器件包： pyocd pack install stm32f407ig'
    exit 1
}

if ($Build) {
    Write-Host '[1/2] 编译 ...' -ForegroundColor Yellow

    if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
        Write-Host '[X] 找不到 cmake' -ForegroundColor Red
        Write-Host '    先跑一次  .\tools\setup_env.ps1  然后重开终端'
        exit 1
    }

    # build 目录还没配置过（或者被删了）就先配一次，
    # 否则 cmake --build 会报 "build is not a directory"
    $buildDir = Join-Path $root 'build'
    if (-not (Test-Path (Join-Path $buildDir 'CMakeCache.txt'))) {
        Write-Host '      还没配置过 CMake，先配置一次 ...' -ForegroundColor DarkYellow
        & cmake -S $root -B $buildDir -G Ninja -DCMAKE_BUILD_TYPE=Debug
        if ($LASTEXITCODE -ne 0) {
            Write-Host '[X] CMake 配置失败，已停止' -ForegroundColor Red
            exit 1
        }
    }

    # --parallel 不带数字 = CMake 自动按本机可用核心数并行
    & cmake --build $buildDir --parallel
    if ($LASTEXITCODE -ne 0) {
        Write-Host '[X] 编译失败，已停止' -ForegroundColor Red
        exit 1
    }
}

if (-not (Test-Path $hex)) {
    Write-Host ('[X] 找不到 ' + $hex) -ForegroundColor Red
    Write-Host '    先编译，或用  .\tools\flash.ps1 -Build'
    exit 1
}

Write-Host '[2/2] 烧录（pyocd + DAPLink）...' -ForegroundColor Yellow

# pyocd 会自动选中唯一的 CMSIS-DAP 探针
& pyocd flash -t $target $hex
$ok = ($LASTEXITCODE -eq 0)
if ($ok) {
    # 烧完复位，让新固件立刻跑起来
    & pyocd reset -t $target | Out-Null
}

if (-not $ok) {
    Write-Host ''
    Write-Host '[X] 烧录失败，按顺序检查：' -ForegroundColor Red
    Write-Host '      1. DAPLink 调试器插好没（USB 认到 CMSIS-DAP 才可用）   pyocd list'
    Write-Host '      2. 板子单独供电没（C 板走 XT30 / USB，DAPLink 供不起电机和 IMU 恒温）'
    Write-Host '      3. SWCLK / SWDIO / GND 接对没'
    Write-Host '      4. STM32CubeIDE / CubeProgrammer / 另一个 VSCode / pyocd 进程占着它没'
    Write-Host '      5. 单独测一下： pyocd reset -t stm32f407ig'
    exit 1
}

Write-Host ''
Write-Host '[OK] 烧录完成' -ForegroundColor Green