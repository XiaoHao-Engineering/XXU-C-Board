# 检查 ST-Link 能不能连上目标板（只连接，不擦除不写入，安全）
$root = Split-Path -Parent $PSScriptRoot

Write-Host ''
Write-Host '===== ST-Link 连接检查 =====' -ForegroundColor Cyan

if (-not (Get-Command STM32_Programmer_CLI -ErrorAction SilentlyContinue)) {
    Write-Host '[X] 找不到 STM32_Programmer_CLI' -ForegroundColor Red
    Write-Host '    先跑一次  .\tools\setup_env.ps1  然后重开终端'
    exit 1
}

$ok = $false
foreach ($m in @('mode=UR', 'mode=normal')) {
    Write-Host ''
    Write-Host ('--- 尝试 ' + $m + ' ---') -ForegroundColor Yellow
    & STM32_Programmer_CLI -c port=SWD $m
    if ($LASTEXITCODE -eq 0) { $ok = $true; break }
    Write-Host '    这种连接方式没成功' -ForegroundColor DarkYellow
}

Write-Host ''
if ($ok) {
    Write-Host '[OK] 连上了 —— 硬件通路没问题' -ForegroundColor Green
    Write-Host '     请重点看上面输出的 "Voltage" 一行：'
    Write-Host '       ~3.3V  -> 正常'
    Write-Host '       <2.5V  -> 板子供电不足'
    Write-Host '       0V     -> 板子根本没上电'
    Write-Host '     下一步：按 F5 调试。若还失败，把 gdbClient_log.txt 发出来。'
} else {
    Write-Host '[X] 连不上，按顺序查：' -ForegroundColor Red
    Write-Host '      1. ST-Link 插好没（看它的指示灯）'
    Write-Host '      2. 板子单独供电没 —— ST-Link 供不起 A 板，这是最常见的'
    Write-Host '      3. SWCLK / SWDIO / GND 接对没，最好再接 3.3V 参考脚'
    Write-Host '      4. STM32CubeIDE / CubeProgrammer / 另一个 VSCode 占着它没'
    Write-Host '      5. 设备管理器里有没有 STMicroelectronics STLink dongle（驱动）'
    Write-Host '      6. ST-Link 固件太旧 —— 上面输出里会带固件版本'
    Write-Host ''
    Write-Host '    上面完整输出就是诊断依据，整段复制出来。' -ForegroundColor DarkYellow
    exit 1
}
