cmake_minimum_required(VERSION 3.16)
project(bms_usb_reader_host_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Só os módulos puros (sem ESP-IDF) de firmware/main
set(MAIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../main)

add_executable(host_tests
    test_main.cpp
    ${MAIN_DIR}/BmsEvpvParser.cpp
    ${MAIN_DIR}/BmsStatusFrame.cpp
)
target_include_directories(host_tests PRIVATE ${MAIN_DIR})

enable_testing()
add_test(NAME host_tests COMMAND host_tests)
