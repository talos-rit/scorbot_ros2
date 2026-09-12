cmake_minimum_required(VERSION 3.16)
project(scorbot_esp_sim CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(scorbot_protocol REQUIRED)

add_library(scorbot_esp_sim_lib src/controller_sim.cpp)
target_include_directories(scorbot_esp_sim_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(scorbot_esp_sim_lib PUBLIC scorbot_protocol::scorbot_protocol)

add_executable(scorbot_esp_sim src/main.cpp)
target_link_libraries(scorbot_esp_sim scorbot_esp_sim_lib util)

install(TARGETS scorbot_esp_sim_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS scorbot_esp_sim RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_controller_sim test/test_controller_sim.cpp)
  target_link_libraries(test_controller_sim scorbot_esp_sim_lib)
  ament_add_gtest(test_sim_link test/test_sim_link.cpp)
  target_link_libraries(test_sim_link scorbot_esp_sim_lib)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(scorbot_protocol)
ament_package()
