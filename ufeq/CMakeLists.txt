cmake_minimum_required(VERSION 3.16)
project(ufeq C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(ufeq
    src/ufeq_pipeline.c
    src/ufeq_validate.c
    src/ufeq_agc.c
    src/ufeq_cfo.c
    src/ufeq_channel.c
    src/ufeq_ruu.c
    src/ufeq_matrix.c
    src/ufeq_equalizer.c
    src/ufeq_idft.c
    src/ufeq_data_cfo.c
    src/ufeq_layer.c
    src/ufeq_uci_demux.c
    src/ufeq_demod.c
    src/ufeq_descramble.c
    src/ufeq_low_papr.c
    src/ufeq_dmrs.c
    src/ufeq_fixed.c
    src/ufeq_tables.c
    platform/ufeq_platform_ref.c
    platform/ufeq_platform_dsp.c
)

target_include_directories(ufeq
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if (MSVC)
    target_compile_options(ufeq PRIVATE /W3)
else()
    target_compile_options(ufeq PRIVATE -Wall -Wextra -Wno-unused-parameter)
    target_link_libraries(ufeq PUBLIC m)
endif()

enable_testing()
add_executable(test_smoke tests/test_smoke.c)
target_link_libraries(test_smoke PRIVATE ufeq)
target_include_directories(test_smoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME smoke COMMAND test_smoke)

add_executable(test_modules tests/test_modules.c)
target_link_libraries(test_modules PRIVATE ufeq)
target_include_directories(test_modules PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME modules COMMAND test_modules)

add_executable(test_extended tests/test_extended.c)
target_link_libraries(test_extended PRIVATE ufeq)
target_include_directories(test_extended PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME extended COMMAND test_extended)

add_executable(test_all tests/test_all.c)
target_link_libraries(test_all PRIVATE ufeq)
target_include_directories(test_all PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME all COMMAND test_all)
