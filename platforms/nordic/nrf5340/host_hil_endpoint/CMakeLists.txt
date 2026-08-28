cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(asr_fc_host_hil_endpoint C)

set(ASR_FC_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../..)

target_sources(app PRIVATE
    src/main.c
    ${ASR_FC_ROOT}/application/protocol/hil_link.c
    ${ASR_FC_ROOT}/application/protocol/motor_link.c
    ${ASR_FC_ROOT}/application/flight/flight_core.c
    ${ASR_FC_ROOT}/modules/controlSystems/src/pid.c
    ${ASR_FC_ROOT}/modules/controlSystems/src/mixer.c
    ${ASR_FC_ROOT}/modules/attitudeMathLibrary/src/quaternion.c)

target_include_directories(app PRIVATE
    ${ASR_FC_ROOT}/include
    ${ASR_FC_ROOT}/modules/controlSystems/include
    ${ASR_FC_ROOT}/modules/attitudeMathLibrary/include)
