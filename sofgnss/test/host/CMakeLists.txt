# Host build of the portable core plus its tests.
#   cmake -S test/host -B build-host && cmake --build build-host && build-host/sofgnss_tests
cmake_minimum_required(VERSION 3.16)
project(sofgnss_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CORE ${CMAKE_CURRENT_SOURCE_DIR}/../../components/core)

add_executable(sofgnss_tests
    ${CORE}/src/frame_parser.cpp
    ${CORE}/src/msg_queue.cpp
    ${CORE}/src/pvdb.cpp
    ${CORE}/src/timing_reporter.cpp
    ${CORE}/src/sof_spin_source.cpp
    ${CORE}/src/ota_receiver.cpp
    ${CORE}/src/cal_signal.cpp
    hal_sim.cpp
    test_main.cpp
    test_frame_parser.cpp
    test_msg_queue.cpp
    test_unwrap.cpp
    test_pvdb.cpp
    test_timing_reporter.cpp
    test_sof_spin.cpp
    test_ota_receiver.cpp
    test_sof_schedule.cpp
    test_cal_signal.cpp
    test_pps_filter.cpp
)
target_include_directories(sofgnss_tests PRIVATE ${CORE}/include ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(sofgnss_tests PRIVATE -Wall -Wextra -g)

enable_testing()
add_test(NAME core COMMAND sofgnss_tests)
