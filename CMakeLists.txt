set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.22)
# 设置目标处理器架构,芯片型号,和目标板简称
set(CMAKE_SYSTEM_PROCESSOR cortex-m0plus)
set(Target_Board MSPM0xx)
set(Board_Name mspm0g3507)

if(NOT DEFINED ARM_GCC_TOOLCHAIN_ROOT_DIR)
    if(DEFINED ENV{ARM_GCC_ROOT} AND NOT "$ENV{ARM_GCC_ROOT}" STREQUAL "")
        file(TO_CMAKE_PATH "$ENV{ARM_GCC_ROOT}" ARM_GCC_TOOLCHAIN_ROOT_DIR)
    else()
        message(FATAL_ERROR
            "Set ARM_GCC_ROOT or ARM_GCC_TOOLCHAIN_ROOT_DIR to the Arm GNU Toolchain directory.")
    endif()
endif()
if("${ARM_GCC_TOOLCHAIN_ROOT_DIR}" STREQUAL "")
    message(FATAL_ERROR
        "ARM_GCC_TOOLCHAIN_ROOT_DIR is empty; set ARM_GCC_ROOT or the CMake cache variable.")
endif()

# 指定交叉编译工具链arm-none-eabi-gcc
set(CMAKE_C_COMPILER "${ARM_GCC_TOOLCHAIN_ROOT_DIR}/bin/arm-none-eabi-gcc.exe")
set(CMAKE_ASM_COMPILER "${ARM_GCC_TOOLCHAIN_ROOT_DIR}/bin/arm-none-eabi-gcc.exe")
set(CMAKE_AR "${ARM_GCC_TOOLCHAIN_ROOT_DIR}/bin/arm-none-eabi-ar.exe")
set(CMAKE_LINKER "${ARM_GCC_TOOLCHAIN_ROOT_DIR}/bin/arm-none-eabi-ld.exe")
set(CMAKE_OBJCOPY "${ARM_GCC_TOOLCHAIN_ROOT_DIR}/bin/arm-none-eabi-objcopy.exe")
set(CMAKE_OBJDUMP "${ARM_GCC_TOOLCHAIN_ROOT_DIR}/bin/arm-none-eabi-objdump.exe")
set(CMAKE_SIZE "${ARM_GCC_TOOLCHAIN_ROOT_DIR}/bin/arm-none-eabi-size.exe")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 项目名称
project(MSPM0 C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
# 添加关于硬件浮点的支持的相关编译选项
message(STATUS "${Target_Board}")
add_compile_options(-mfloat-abi=soft)
add_link_options(-mfloat-abi=soft)

# 指定芯片型号的相关编译选项
add_compile_options(-mcpu=${CMAKE_SYSTEM_PROCESSOR} -mthumb)
add_link_options(-mcpu=${CMAKE_SYSTEM_PROCESSOR} -mthumb)

add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
add_link_options(-specs=nosys.specs -lc -lm)
add_link_options(-u _printf_float)


# 进行汇编文件的预编译
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)
# 提供编译选项,根据不同的编译类型进行不同的优化,常见的编译类型有Debug,Release,RelWithDebInfo,MinSizeRel
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()


# 添加链接脚本
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/linker/mspm0g3507.ld)

# 添加链接选项,生成map文件
add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-T ${LINKER_SCRIPT})


# 可选编译开关：TI Drivers / 逐飞库
option(USE_TI_DRIVERS     "编译 TI Drivers 源码" OFF)
option(USE_ZF_LIB         "编译逐飞库 (zf) 源码" OFF)

set(LINK_HEAP_SIZE 0x800)
set(LINK_STACK_SIZE 0x800)

add_link_options(
    -Wl,--defsym=__heap_size__=${LINK_HEAP_SIZE}
    -Wl,--defsym=__stack_size__=${LINK_STACK_SIZE}
)

# 递归收集内核、驱动和用户代码
file(
    GLOB_RECURSE PROJECT_SOURCE_FILES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/source/core/startup_mspm0g350x_gcc.c
    ${CMAKE_SOURCE_DIR}/source/core/ti/driverlib/*.c
    ${CMAKE_SOURCE_DIR}/user/*.c)

# 根据开关决定是否收集 TI Drivers 源码
if(USE_TI_DRIVERS)
    file(GLOB_RECURSE TI_DRIVERS_SOURCE_FILES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/source/core/ti/drivers/*.c)
    list(APPEND PROJECT_SOURCE_FILES ${TI_DRIVERS_SOURCE_FILES})
    message(STATUS "TI Drivers: 已启用")
else()
    message(STATUS "TI Drivers: 已禁用")
endif()

# 根据开关决定是否收集逐飞库源码
if(USE_ZF_LIB)
    file(GLOB_RECURSE ZF_SOURCE_FILES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/source/core/zf/*.c)
    list(APPEND PROJECT_SOURCE_FILES ${ZF_SOURCE_FILES})
    message(STATUS "逐飞库 (zf): 已启用")
else()
    message(STATUS "逐飞库 (zf): 已禁用")
endif()

list(SORT PROJECT_SOURCE_FILES)

# 生成可执行文件
add_executable(${PROJECT_NAME}.elf ti_msp_dl_config.c ${PROJECT_SOURCE_FILES} ${LINKER_SCRIPT})

set(COMMON_INCLUDE_DIRECTORIES
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/include/core
        ${CMAKE_SOURCE_DIR}/include/core/ti/driverlib
        ${CMAKE_SOURCE_DIR}/include/core/third_party/CMSIS/Core/Include
        ${CMAKE_SOURCE_DIR}/user
        ${CMAKE_SOURCE_DIR}/user/code
        ${CMAKE_SOURCE_DIR}/user/bsp
)

# 根据开关添加对应的头文件目录
if(USE_ZF_LIB)
    list(APPEND COMMON_INCLUDE_DIRECTORIES ${CMAKE_SOURCE_DIR}/include/core/zf)
endif()

include_directories(${CMAKE_SOURCE_DIR})

function(apply_common_target_settings target_name)
    target_include_directories(${target_name} PUBLIC ${COMMON_INCLUDE_DIRECTORIES})
endfunction()

apply_common_target_settings(${PROJECT_NAME}.elf)
target_link_libraries(${PROJECT_NAME}.elf PRIVATE m)

# 编译宏定义
add_definitions(
        -D__MSPM0G3507__  # 定义芯片型号
)

# 定义其他类型的可执行文件
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
# 生成hex和bin文件
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
