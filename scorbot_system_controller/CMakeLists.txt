cmake_minimum_required(VERSION 3.16)
project(scorbot_system_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_DEPENDS
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  scorbot_msgs
  std_srvs)

find_package(ament_cmake REQUIRED)
foreach(dep ${THIS_PACKAGE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()
find_package(scorbot_protocol REQUIRED)

add_library(scorbot_system_controller SHARED src/scorbot_system_controller.cpp)
target_include_directories(scorbot_system_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(scorbot_system_controller PUBLIC ${THIS_PACKAGE_DEPENDS})
target_link_libraries(scorbot_system_controller PUBLIC scorbot_protocol::scorbot_protocol)
pluginlib_export_plugin_description_file(controller_interface scorbot_system_controller_plugin.xml)

install(TARGETS scorbot_system_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPENDS} scorbot_protocol)
ament_package()
