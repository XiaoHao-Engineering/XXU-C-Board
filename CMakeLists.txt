# ============================================================================
#  XXU-RoboMaster-Sentry   新乡学院 RoboMaster 哨兵底盘框架（C 板 / STM32F407IGH6）
# ============================================================================
#  注意：CubeMX 每次 GENERATE CODE 都会用模板覆盖本文件。
#        重新生成后请执行： powershell -File tools/cubemx_postgen.ps1
#        基准副本： cmake/CMakeLists.app.txt
# ============================================================================

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.20)

# 交叉编译工具链
set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR           arm-none-eabi-ar)
set(CMAKE_OBJCOPY      arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP      arm-none-eabi-objdump)
set(SIZE               arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(XXU-RoboMaster-Sentry C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
endif()

# 不用硬件浮点，省掉开 FPU 的初始化，避免 NoCP 异常
# ⚠️ CMSIS-DSP 必须跟着源码一起编（Drivers/CMSIS/DSP/Source）：
#    Middlewares 里的 libarm_cortexM4lf_math.a 是【硬件浮点 ABI】的预编译库，
#    和这里的软浮点工程 ABI 不兼容，arm_sin_cos_f32 / arm_sin_f32 会取错参数。
add_compile_options(-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-Ofast)
elseif (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    add_compile_options(-Ofast -g)
elseif (CMAKE_BUILD_TYPE STREQUAL "MinSizeRel")
    add_compile_options(-Os)
else ()
    add_compile_options(-O2 -g)
endif ()

include_directories(
        Core/Inc
        Drivers/STM32F4xx_HAL_Driver/Inc
        Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32F4xx/Include
        Drivers/CMSIS/Include
        Drivers/CMSIS/DSP/Include
        Drivers/CMSIS/DSP/PrivateInclude
)

add_definitions(-DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DARM_MATH_CM4 -D__FPU_PRESENT=1U)

# ⚠️ CONFIGURE_DEPENDS：让 CMake 每次构建前重新扫描目录。
#    没有它的话，新增 .c 文件后必须手动"重新配置 CMake"才会被编译进来，
#    症状是"文件明明写了却没生效"，非常难查。
#    只对【你自己会增删文件的目录】用它；驱动库那棵树不适用（见下面）。
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
        "User_Drives/*.*"
        "User_Algorithm/*.*"
        "User_Architect/*.*"
        "User_Application/*.*"
        "User_Lib/*.*"
        "SEGGER_RTT/*.*"
        "Core/*.*")

# ---------------------------------------------------------------------------
#  ★★ CMSIS-DSP 只编译实际用到的几个文件 —— 这是编译速度的关键，别改回整树 GLOB
#
#  整棵 Drivers/CMSIS/DSP/Source 有 564 个 .c，而本工程实际只用 5 个函数：
#      arm_fir_init_f32 / arm_fir_f32                   <- user_fir.c 的 FIR 滤波
#      arm_dot_prod_f32 / arm_sin_f32 / arm_sin_cos_f32 <- user_coord.c 的坐标变换
#  以前把整树 GLOB 进来，644 个编译单元里 564 个是白编的，
#  全量编译因此从几十秒变成好几分钟。改完是 644 -> 87 个编译单元。
#
#  ★ 以后要用别的 DSP 函数（arm_sqrt_f32、FFT、矩阵运算……）：
#    1) 把该函数所在的 .c 按路径加进下面这个竖表；
#    2) 如果报 "undefined reference to arm_xxx"，就是这个原因，照抄路径补上。
#    （部分函数还依赖 arm_common_tables.c 里的查找表，已经在表里了。）
# ---------------------------------------------------------------------------
set(CMSIS_DSP_SRC
        Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_dot_prod_f32.c
        Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sin_f32.c
        Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_cos_f32.c
        Drivers/CMSIS/DSP/Source/ControllerFunctions/arm_sin_cos_f32.c
        Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_f32.c
        Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_init_f32.c
        Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c
        # 矩阵运算：user_quaternion_ekf / user_kalman_filter 用（四元数 EKF 的姿态解算）
        Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f32.c
        Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f32.c
        Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f32.c
        Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.c
        Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.c
        Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c
        Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f32.c
)

# 驱动库单独扫：不加 CONFIGURE_DEPENDS（这棵树不会变，省掉每次构建的目录扫描），
# 并把 CMSIS-DSP 的 Source 排除掉（改由上面的显式列表提供）。
file(GLOB_RECURSE DRIVER_SOURCES "Drivers/*.*")
list(FILTER DRIVER_SOURCES EXCLUDE REGEX "CMSIS.DSP.Source.")
list(APPEND SOURCES ${DRIVER_SOURCES} ${CMSIS_DSP_SRC})

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F407xx_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m4 -mthumb)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

# 数学库：姿态解算用 sqrtf / atan2f
target_link_libraries(${PROJECT_NAME}.elf m)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Generating ${HEX_FILE} and ${BIN_FILE}")
