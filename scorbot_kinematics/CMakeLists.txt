cmake_minimum_required(VERSION 3.16)
project(scorbot_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# Header-only closed-form solver (no ROS dependencies).
add_library(analytic_ik INTERFACE)
target_include_directories(analytic_ik INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# MoveIt plugin.
add_library(scorbot_kinematics_plugin SHARED src/scorbot_kinematics_plugin.cpp)
target_include_directories(scorbot_kinematics_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(scorbot_kinematics_plugin
  rclcpp moveit_core pluginlib geometry_msgs moveit_msgs tf2_eigen)
target_link_libraries(scorbot_kinematics_plugin Eigen3::Eigen)

pluginlib_export_plugin_description_file(moveit_core scorbot_kinematics_plugin_description.xml)

install(TARGETS scorbot_kinematics_plugin analytic_ik
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_analytic_ik test/test_analytic_ik.cpp)
  target_link_libraries(test_analytic_ik analytic_ik)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp moveit_core pluginlib geometry_msgs moveit_msgs tf2_eigen Eigen3)
ament_package()
