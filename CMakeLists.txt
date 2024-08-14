cmake_minimum_required(VERSION 3.1)
project(thrusters_blue_robotics_t500 VERSION 0.0)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.orogen/config")
include(thrusters_blue_robotics_t500Base)

if(ROCK_TEST_ENABLED)
    enable_testing()
    find_package(Syskit REQUIRED)
    syskit_orogen_tests(test)
endif()