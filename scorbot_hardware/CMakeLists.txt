cmake_minimum_required(VERSION 3.16)
project(scorbot_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(scorbot_protocol REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

# Everything the interface does, without ROS: link client, calibration, session.
# Tested in-process against scorbot_esp_sim; usable from a plain g++ too.
add_library(scorbot_hardware_core SHARED
  src/controller_client.cpp
  src/calibration.cpp
  src/session.cpp)
target_include_directories(scorbot_hardware_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(scorbot_hardware_core PUBLIC
  scorbot_protocol::scorbot_protocol
  yaml-cpp::yaml-cpp
  Threads::Threads)

# The ros2_control plugin: a thin adapter over the core.
add_library(scorbot_hardware SHARED src/scorbot_system.cpp)
target_link_libraries(scorbot_hardware PUBLIC scorbot_hardware_core)
ament_target_dependencies(scorbot_hardware PUBLIC hardware_interface pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface scorbot_hardware_plugin.xml)

install(TARGETS scorbot_hardware_core scorbot_hardware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(scorbot_esp_sim REQUIRED)
  find_package(scorbot_description REQUIRED)  # shipped calibration files

  # scorbot_description_DIR is <prefix>/share/scorbot_description/cmake.
  ament_add_gtest(test_calibration test/test_calibration.cpp
    ENV SCORBOT_CALIBRATION_DIR=${scorbot_description_DIR}/../config)
  target_link_libraries(test_calibration scorbot_hardware_core)

  ament_add_gtest(test_controller_client test/test_controller_client.cpp)
  target_link_libraries(test_controller_client scorbot_hardware_core scorbot_esp_sim::scorbot_esp_sim_lib)

  ament_add_gtest(test_session test/test_session.cpp TIMEOUT 180)
  target_link_libraries(test_session scorbot_hardware_core scorbot_esp_sim::scorbot_esp_sim_lib)

  # The plugin itself, loaded through the ResourceManager and driven against the
  # simulator executable over a pty (what robot.launch.py mock:=false does).
  find_package(ament_index_cpp REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  ament_add_gtest(test_scorbot_system test/test_scorbot_system.cpp TIMEOUT 180)
  target_link_libraries(test_scorbot_system scorbot_hardware)
  ament_target_dependencies(test_scorbot_system hardware_interface rclcpp ament_index_cpp ros2_control_test_assets)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle scorbot_protocol yaml_cpp_vendor yaml-cpp)
ament_package()
