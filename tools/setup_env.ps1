# 把 STM32Cube 工具链加进 PATH，供命令行和 VSCode 使用
#   .\tools\setup_env.ps1            加到【用户级】永久 PATH（推荐，只需跑一次）
#   .\tools\setup_env.ps1 -Session   只对当前窗口生效，关掉就没了

param([switch]$Session)

$ErrorActionPreference = 'Stop'

$root = Join-Path $env:LOCALAPPDATA 'stm32cube\bundles'

$dirs = @(
    (Join-Path $root 'gnu-tools-for-stm32\14.3.1+st.2\bin'),   # arm-none-eabi-gcc / g++ / objcopy
    (Join-Path $root 'cmake\4.3.1+st.1\bin'),                  # cmake
    (Join-Path $root 'ninja\1.13.2+st.1\bin'),                 # ninja
    (Join-Path $root 'gnu-gdb-for-stm32\14.3.1+st.2\bin'),     # arm-none-eabi-gdb
    (Join-Path $root 'programmer\2.23.0\bin'),                 # STM32_Programmer_CLI
    (Join-Path $root 'stlink-gdbserver\7.14.0+st.2\bin')       # ST-LINK_gdbserver
)

$missing = @()
foreach ($d in $dirs) {
    if (-not (Test-Path $d)) { $missing += $d }
}
if ($missing.Count -gt 0) {
    Write-Host '以下目录不存在，请先在 VSCode 里装好 STM32 VS Code Extension 并让它下载工具包：'
    $missing | ForEach-Object { Write-Host ('  ' + $_) }
    exit 1
}

if ($Session) {
    $env:PATH = ($dirs -join ';') + ';' + $env:PATH
    Write-Host '已加入当前会话 PATH：'
    $dirs | ForEach-Object { Write-Host ('  ' + $_) }
} else {
    $old = [Environment]::GetEnvironmentVariable('PATH', 'User')
    if ($null -eq $old) { $old = '' }
    $parts = $old -split ';' | Where-Object { $_ -ne '' }
    $added = @()
    foreach ($d in $dirs) {
        if ($parts -notcontains $d) { $parts += $d; $added += $d }
    }
    if ($added.Count -eq 0) {
        Write-Host '用户 PATH 里已经有全部目录，无需改动。'
    } else {
        [Environment]::SetEnvironmentVariable('PATH', ($parts -join ';'), 'User')
        Write-Host '已写入用户级 PATH（新开终端生效）：'
        $added | ForEach-Object { Write-Host ('  + ' + $_) }
    }
}

Write-Host ''
Write-Host '校验：'
foreach ($cmd in 'arm-none-eabi-gcc', 'cmake', 'ninja') {
    $p = Join-Path ($dirs | Where-Object { Test-Path (Join-Path $_ ($cmd + '.exe')) } | Select-Object -First 1) ($cmd + '.exe')
    if ($p -and (Test-Path $p)) { Write-Host ('  ' + $cmd + '  OK') }
    else { Write-Host ('  ' + $cmd + '  未找到') }
}
