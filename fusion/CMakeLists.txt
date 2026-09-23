cmake_minimum_required(VERSION 3.20)
project(otolith_fusion CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(Catch2 3 REQUIRED)

add_library(otolith_leg src/leg_kin.cpp)
target_include_directories(otolith_leg PUBLIC include)
target_link_libraries(otolith_leg PUBLIC Eigen3::Eigen)
target_compile_options(otolith_leg PRIVATE -Wall -Wextra -Wpedantic)

add_library(otolith_log src/log.cpp src/estimate_log.cpp)
target_include_directories(otolith_log PUBLIC include)
target_link_libraries(otolith_log PUBLIC Eigen3::Eigen)
target_compile_options(otolith_log PRIVATE -Wall -Wextra -Wpedantic)

add_library(otolith_fusion src/fusion.cpp)
target_include_directories(otolith_fusion PUBLIC include)
target_link_libraries(otolith_fusion PUBLIC otolith_leg otolith_log Eigen3::Eigen)
target_compile_options(otolith_fusion PRIVATE -Wall -Wextra -Wpedantic)

# --- tests ---
include(CTest)
include(Catch)

add_executable(test_log tests/test_log.cpp)
target_link_libraries(test_log PRIVATE otolith_log Catch2::Catch2WithMain)
catch_discover_tests(test_log)

add_executable(log_check tests/log_check.cpp)
target_link_libraries(log_check PRIVATE otolith_log)

add_executable(test_fusion tests/test_fusion.cpp)
target_link_libraries(test_fusion PRIVATE otolith_fusion Catch2::Catch2WithMain)
catch_discover_tests(test_fusion)

add_executable(test_jitter tests/test_jitter.cpp)
target_link_libraries(test_jitter PRIVATE otolith_fusion Catch2::Catch2WithMain)
catch_discover_tests(test_jitter)

add_executable(fuse_log src/fuse_log.cpp)
target_link_libraries(fuse_log PRIVATE otolith_fusion otolith_log)

# --- v0.2 transport bake-off bench (ADR-0005) ---
# Bench binaries, not tests: timing-sensitive, never in ctest.
find_package(Threads REQUIRED)

add_executable(bench_c bench/bench_c.cpp)
target_include_directories(bench_c PRIVATE bench include)
target_compile_options(bench_c PRIVATE -Wall -Wextra -Wpedantic)

add_executable(bench_d bench/bench_d.cpp)
target_include_directories(bench_d PRIVATE bench include)
target_link_libraries(bench_d PRIVATE Threads::Threads)
target_compile_options(bench_d PRIVATE -Wall -Wextra -Wpedantic)

# Contender A needs the ROS 2 edge stack (same CMAKE_PREFIX_PATH workaround
# as the colcon build: -DCMAKE_PREFIX_PATH=$CONDA_PREFIX).
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(bench_a_pub bench/bench_a_pub.cpp)
target_include_directories(bench_a_pub PRIVATE bench include)
target_link_libraries(bench_a_pub PRIVATE
  rclcpp::rclcpp std_msgs::std_msgs__rosidl_typesupport_cpp)
target_compile_options(bench_a_pub PRIVATE -Wall -Wextra -Wpedantic)

add_executable(bench_a_sub bench/bench_a_sub.cpp)
target_include_directories(bench_a_sub PRIVATE bench include)
target_link_libraries(bench_a_sub PRIVATE
  rclcpp::rclcpp std_msgs::std_msgs__rosidl_typesupport_cpp Threads::Threads)
target_compile_options(bench_a_sub PRIVATE -Wall -Wextra -Wpedantic)
