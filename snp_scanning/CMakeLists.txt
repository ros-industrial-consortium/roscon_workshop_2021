cmake_minimum_required(VERSION 3.5)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(noether_tpp REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(ROS2_DEPS
    industrial_reconstruction_msgs
    visualization_msgs
    rclpy
    rclcpp
    snp_msgs
    tf2_ros)
foreach(dep ${ROS2_DEPS})
  find_package(${dep} REQUIRED)
endforeach()
find_package(ament_cmake REQUIRED)

add_executable(scan_trajectory_node src/scan_trajectory_node.cpp)
target_link_libraries(scan_trajectory_node noether::noether_tpp Eigen3::Eigen)
ament_target_dependencies(
  scan_trajectory_node
  rclcpp
  snp_msgs
  tf2_ros)
target_include_directories(scan_trajectory_node PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                       "$<INSTALL_INTERFACE:include>")

install(TARGETS scan_trajectory_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Python executables
install(PROGRAMS scripts/reconstruction_sim_node scripts/scan_motion_plan_from_file_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
