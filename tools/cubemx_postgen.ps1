# CubeMX 重新生成代码后运行本脚本：
#   1. 把被模板覆盖的 CMakeLists.txt 还原成工程实际用的版本
#   2. 检查 USER CODE 钩子有没有丢
$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
$src = Join-Path $root 'cmake\CMakeLists.app.txt'
$dst = Join-Path $root 'CMakeLists.txt'

if (-not (Test-Path $src)) {
    Write-Host ('找不到基准文件: ' + $src)
    exit 1
}
Copy-Item $src $dst -Force
Write-Host '已恢复 CMakeLists.txt'

$checks = @(
    @{ File = 'Core\Src\stm32f4xx_it.c'; Pattern = 'DBUS_Handler\(&huart3\)'; Desc = 'USART3 中断里调用 DBUS_Handler（C 板 DBUS 在 USART3）' },
    @{ File = 'Core\Src\stm32f4xx_it.c'; Pattern = 'SysTick_Handle\(\)';      Desc = 'SysTick 里调用 SysTick_Handle' },
    @{ File = 'Core\Src\stm32f4xx_it.c'; Pattern = 'TIM1_UP_TIM10_IRQHandler'; Desc = 'TIM10 更新中断（JScope 10kHz 节拍）' },
    @{ File = 'Core\Src\tim.c';          Pattern = 'TIM1_UP_TIM10_IRQn';      Desc = 'TIM10 的 NVIC 使能（CubeMX 里要勾 TIM1_UP_TIM10_IRQn）' },
    @{ File = 'Core\Src\main.c';         Pattern = 'ChassisControl_Init';     Desc = 'main 里初始化底盘控制' },
    @{ File = 'Core\Src\main.c';         Pattern = 'TIMER_RegisterCallback';  Desc = 'main 里挂 JScope 回调' },
    @{ File = 'Core\Src\main.c';         Pattern = 'DJI_Motor_Init';          Desc = 'main 里初始化底盘电机' },
    @{ File = 'Core\Src\main.c';         Pattern = 'BMI088_Init';             Desc = 'main 里初始化板载 BMI088' },
    @{ File = 'Core\Src\bsp.c';          Pattern = 'user_chassis';            Desc = 'bsp.c 里的全局实例表' },
    @{ File = 'Core\Inc\main.h';         Pattern = 'LED_RED_Pin';             Desc = 'main.h 里的 LED 引脚标签（PH12/PH11/PH10）' },
    @{ File = 'Core\Inc\main.h';         Pattern = 'IMU_CS_ACCEL_Pin';        Desc = 'main.h 里的 BMI088 片选标签（PA4/PB0）' },
    @{ File = 'Drivers\CMSIS\DSP\Include\arm_math.h'; Pattern = 'arm_math';   Desc = 'CMSIS-DSP 源码树在（软浮点必须跟源码一起编）' }
)

$bad = 0
foreach ($c in $checks) {
    $p = Join-Path $root $c.File
    if (-not (Test-Path $p)) {
        Write-Host ('  缺失文件 ' + $c.File)
        $bad++
        continue
    }
    if (Select-String -Path $p -Pattern $c.Pattern -Quiet) {
        Write-Host ('  OK    ' + $c.Desc)
    } else {
        Write-Host ('  丢了  ' + $c.Desc + '   -> 需要手工补回 ' + $c.File)
        $bad++
    }
}

if ($bad -gt 0) {
    Write-Host ''
    Write-Host ('有 ' + $bad + ' 项需要处理，请看 docs\C板移植说明.md')
    exit 1
}
Write-Host '全部检查通过'
