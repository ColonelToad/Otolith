cmake_minimum_required(VERSION 3.20)
project(otolith_fusion CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(Catch2 3 REQUIRED)

add_library(otolith_log src/log.cpp)
target_include_directories(otolith_log PUBLIC include)
target_link_libraries(otolith_log PUBLIC Eigen3::Eigen)
target_compile_options(otolith_log PRIVATE -Wall -Wextra -Wpedantic)

# --- tests ---
include(CTest)
include(Catch)

add_executable(test_log tests/test_log.cpp)
target_link_libraries(test_log PRIVATE otolith_log Catch2::Catch2WithMain)
catch_discover_tests(test_log)

add_executable(log_check tests/log_check.cpp)
target_link_libraries(log_check PRIVATE otolith_log)
