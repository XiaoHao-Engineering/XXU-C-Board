# 烧录固件到 STM32（像 CCS 的烧录按钮）
#   .\tools\flash.ps1          只烧录
#   .\tools\flash.ps1 -Build   先编译再烧录
param([switch]$Build)

$root = Split-Path -Parent $PSScriptRoot
$hex  = Join-Path $root 'build\XXU-RoboMaster-Sentry.hex'

Write-Host ''
Write-Host '===== XXU-RoboMaster-Sentry =====' -ForegroundColor Cyan

if (-not (Get-Command STM32_Programmer_CLI -ErrorAction SilentlyContinue)) {
    Write-Host '[X] 找不到 STM32_Programmer_CLI' -ForegroundColor Red
    Write-Host '    先跑一次  .\tools\setup_env.ps1  然后重开终端'
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

    # --parallel 不带数字 = CMake 自动按本机可用核心数并行（比写死 -j 8 快得多）
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

Write-Host '[2/2] 烧录 ...' -ForegroundColor Yellow

# 先试“复位下连接”，不行再试普通模式（有些板子没接复位线）
$done = $false
foreach ($mode in @('mode=UR', 'mode=normal')) {
    Write-Host ('      连接方式 ' + $mode)
    & STM32_Programmer_CLI -c port=SWD $mode -w $hex -v -rst
    if ($LASTEXITCODE -eq 0) { $done = $true; break }
    Write-Host '      这种方式没连上，换一种重试 ...' -ForegroundColor DarkYellow
}

if (-not $done) {
    Write-Host ''
    Write-Host '[X] 烧录失败，按顺序检查：' -ForegroundColor Red
    Write-Host '      1. ST-Link 插好没'
    Write-Host '      2. 板子单独供电没（C 板走 XT30 / USB，ST-Link 供不起电机和 IMU 恒温）'
    Write-Host '      3. SWCLK / SWDIO / GND 接对没'
    Write-Host '      4. STM32CubeIDE / CubeProgrammer / 另一个 VSCode 占着它没'
    Write-Host '      5. 单独测一下： STM32_Programmer_CLI -c port=SWD mode=UR'
    exit 1
}

Write-Host ''
Write-Host '[OK] 烧录完成' -ForegroundColor Green
