cmake_minimum_required(VERSION 3.16)
project(asr_fc VERSION 0.1.0 LANGUAGES C)

add_library(asr_fc_motor_link STATIC
    application/protocol/motor_link.c
    application/sil/virtual_motor_bank.c)
target_include_directories(asr_fc_motor_link PUBLIC include)
target_compile_features(asr_fc_motor_link PUBLIC c_std_11)
target_compile_options(asr_fc_motor_link PRIVATE
    $<$<C_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)

if(UNIX)
    add_executable(asr_fc_hil_usb_bridge tools/hil_usb_bridge.c)
    target_link_libraries(asr_fc_hil_usb_bridge PRIVATE asr_fc_motor_link)
    target_compile_options(asr_fc_hil_usb_bridge PRIVATE
        $<$<C_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
endif()

enable_testing()
add_executable(motor_link_tests tests/test_motor_link.c)
target_link_libraries(motor_link_tests PRIVATE asr_fc_motor_link)
target_compile_options(motor_link_tests PRIVATE
    $<$<C_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
add_test(NAME motor_link_tests COMMAND motor_link_tests)

add_executable(virtual_motor_bank_tests tests/test_virtual_motor_bank.c)
target_link_libraries(virtual_motor_bank_tests PRIVATE asr_fc_motor_link)
target_compile_options(virtual_motor_bank_tests PRIVATE
    $<$<C_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
add_test(NAME virtual_motor_bank_tests COMMAND virtual_motor_bank_tests)
