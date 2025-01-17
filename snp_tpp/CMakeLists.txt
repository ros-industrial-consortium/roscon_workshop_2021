cmake_minimum_required(VERSION 3.5)
project(snp_tpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(noether_tpp REQUIRED)
find_package(noether_gui REQUIRED)
find_package(PCL REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_polygon_selection_tool REQUIRED)
find_package(snp_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

# Custom mesh modifier and snp planner
qt5_wrap_ui(${PROJECT_NAME}_ui_mocs ui/roi_selection_mesh_modifier_widget.ui ui/snp_raster_planner_widget.ui)
qt5_wrap_cpp(${PROJECT_NAME}_mocs include/${PROJECT_NAME}/roi_selection_mesh_modifier_widget.h
             include/${PROJECT_NAME}/snp_raster_planner_widget.h)
add_library(
  ${PROJECT_NAME}_mesh_modifier SHARED
  src/roi_selection_mesh_modifier.cpp
  src/roi_selection_mesh_modifier_widget.cpp
  src/snp_raster_planner_widget.cpp
  ${${PROJECT_NAME}_ui_mocs}
  ${${PROJECT_NAME}_mocs})
target_include_directories(
  ${PROJECT_NAME}_mesh_modifier PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                       "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>" "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}_mesh_modifier noether::noether_tpp noether::noether_gui)
ament_target_dependencies(
  ${PROJECT_NAME}_mesh_modifier
  geometry_msgs
  rviz_polygon_selection_tool
  rclcpp
  snp_msgs
  tf2_eigen)

# Custom drawn planner
qt5_wrap_ui(${PROJECT_NAME}_ui_drawn_mocs ui/drawn_toolpath_planner_widget.ui)
qt5_wrap_cpp(${PROJECT_NAME}_drawn_mocs include/${PROJECT_NAME}/drawn_toolpath_planner_widget.h)
add_library(
  ${PROJECT_NAME}_drawn_toolpath_planner SHARED
  src/drawn_toolpath_planner.cpp
  src/drawn_toolpath_planner_widget.cpp
  ${${PROJECT_NAME}_ui_drawn_mocs}
  ${${PROJECT_NAME}_drawn_mocs})
target_include_directories(
  ${PROJECT_NAME}_drawn_toolpath_planner
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
         "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}_drawn_toolpath_planner noether::noether_tpp noether::noether_gui)
ament_target_dependencies(
  ${PROJECT_NAME}_drawn_toolpath_planner
  geometry_msgs
  rviz_polygon_selection_tool
  rclcpp
  snp_msgs
  tf2_eigen)

# Noether plugin library
add_library(${PROJECT_NAME}_plugins SHARED src/plugins.cpp)
target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME}_mesh_modifier ${PROJECT_NAME}_drawn_toolpath_planner)

# Custom TPP widget
qt5_wrap_cpp(${PROJECT_NAME}_widget_mocs include/${PROJECT_NAME}/tpp_widget.h)
add_library(${PROJECT_NAME}_widget SHARED src/tpp_widget.cpp ${${PROJECT_NAME}_widget_mocs})
target_include_directories(
  ${PROJECT_NAME}_widget PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>" "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}_widget noether::noether_gui ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_widget snp_msgs tf2_eigen)
target_compile_definitions(${PROJECT_NAME}_widget PUBLIC SNP_TPP_GUI_PLUGINS="${PROJECT_NAME}_plugins")

# TPP GUI application
add_executable(${PROJECT_NAME}_app src/tpp_app.cpp)
target_link_libraries(${PROJECT_NAME}_app ${PROJECT_NAME}_widget)
ament_target_dependencies(${PROJECT_NAME}_app rclcpp)

# Install the headers
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Install the executables
install(TARGETS ${PROJECT_NAME}_app DESTINATION lib/${PROJECT_NAME})

# Install the library(ies)
install(
  TARGETS ${PROJECT_NAME}_mesh_modifier
          ${PROJECT_NAME}_drawn_toolpath_planner
          ${PROJECT_NAME}_plugins
          ${PROJECT_NAME}_widget
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION lib)
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  noether_tpp
  noether_gui
  PCL
  geometry_msgs
  rviz_polygon_selection_tool
  rclcpp
  snp_msgs
  tf2_eigen)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
