set(CMAKE_SYSTEM_NAME Generic)
cmake_minimum_required(VERSION 3.16)
enable_language(ASM)

if(NOT DEFINED ENV{SdkRootDirPath})
    message(FATAL_ERROR "SdkRootDirPath must reference the composed MCUX SDK")
endif()
set(SdkRootDirPath "$ENV{SdkRootDirPath}")
get_filename_component(AsrRoot "${CMAKE_CURRENT_SOURCE_DIR}/../../../../.."
    ABSOLUTE)
set(ProjDirPath "${CMAKE_CURRENT_SOURCE_DIR}")

project(asr_fc_kv31f_hil_sink C ASM)
set(MCUX_SDK_PROJECT_NAME asr_fc_kv31f_hil_sink.elf)

set(common_flags
    -DCPU_MKV31F512VLL12 -DFRDM_KV31F -DMCUXPRESSO_SDK
    -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
    -ffreestanding -fno-builtin -ffunction-sections -fdata-sections
    -Wall -Wextra -Wpedantic -Werror -std=c11)
add_compile_options(${common_flags})
add_link_options(
    -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
    --specs=nano.specs --specs=nosys.specs
    -Wl,--gc-sections
    -T${SdkRootDirPath}/core/devices/MKV31F51212/gcc/MKV31F512xxx12_flash.ld)

add_executable(${MCUX_SDK_PROJECT_NAME}
    ../src/main.c
    ${AsrRoot}/application/protocol/motor_link.c
    ${SdkRootDirPath}/examples/frdmkv31f/demo_apps/hello_world/pin_mux.c
    ${SdkRootDirPath}/core/boards/frdmkv31f/board.c
    ${SdkRootDirPath}/core/boards/frdmkv31f/clock_config.c)
target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ../src
    ${AsrRoot}/include
    ${SdkRootDirPath}/examples/frdmkv31f/demo_apps/hello_world
    ${SdkRootDirPath}/core/boards/frdmkv31f)

set(CMAKE_MODULE_PATH ${SdkRootDirPath}/core)
include(all_devices)
include(driver_uart)
include(driver_clock)
include(driver_common)
include(driver_gpio)
include(driver_port)
include(device_CMSIS)
include(device_startup)
include(device_system)
include(CMSIS_Include_core_cm)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE m c gcc nosys)
