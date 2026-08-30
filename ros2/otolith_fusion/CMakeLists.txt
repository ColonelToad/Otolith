cmake_minimum_required(VERSION 3.8)
project(otolith_fusion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Reuse the core fusion sources (plain CMake lib) — keep single source of truth
set(FUSION_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../fusion)
add_library(otolith_fusion_core STATIC
  ${FUSION_DIR}/src/fusion.cpp
  ${FUSION_DIR}/src/leg_kin.cpp
  ${FUSION_DIR}/src/log.cpp
  ${FUSION_DIR}/src/estimate_log.cpp
)
target_include_directories(otolith_fusion_core PUBLIC
  ${FUSION_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(otolith_fusion_core Eigen3::Eigen)
target_compile_features(otolith_fusion_core PUBLIC cxx_std_20)

add_executable(fusion_node src/fusion_node.cpp)
target_link_libraries(fusion_node otolith_fusion_core)
ament_target_dependencies(fusion_node rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs)

install(TARGETS fusion_node otolith_fusion_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
