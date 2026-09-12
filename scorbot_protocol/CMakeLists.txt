cmake_minimum_required(VERSION 3.16)
project(scorbot_protocol C CXX)

# Builds inside a ROS 2 workspace (ament) or standalone (plain CMake), so the firmware
# and bench tooling can use it without ROS.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake QUIET)

add_library(scorbot_protocol
  third_party/nanopb/pb_common.c
  third_party/nanopb/pb_encode.c
  third_party/nanopb/pb_decode.c
  generated/scorbot/v1/scorbot.pb.c
  src/messages.cpp
  src/transport.cpp
  src/link.cpp
)
target_include_directories(scorbot_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/generated>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/nanopb>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/scorbot_protocol/generated>
  $<INSTALL_INTERFACE:include/scorbot_protocol/nanopb>)
# nanopb: no malloc, static fields only, 8-bit field counts are enough for our messages.
target_compile_definitions(scorbot_protocol PUBLIC PB_ENABLE_MALLOC=0 PB_FIELD_32BIT=0)
set_target_properties(scorbot_protocol PROPERTIES POSITION_INDEPENDENT_CODE ON)
# Vendored C code is not held to our warning level.
set_source_files_properties(
  third_party/nanopb/pb_common.c third_party/nanopb/pb_encode.c third_party/nanopb/pb_decode.c
  generated/scorbot/v1/scorbot.pb.c
  PROPERTIES COMPILE_OPTIONS "-w")

add_executable(scorbot_protocol_cli src/cli.cpp)
target_link_libraries(scorbot_protocol_cli scorbot_protocol)

install(TARGETS scorbot_protocol scorbot_protocol_cli
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY generated/ DESTINATION include/scorbot_protocol/generated)
install(DIRECTORY third_party/nanopb/ DESTINATION include/scorbot_protocol/nanopb
  FILES_MATCHING PATTERN "*.h" PATTERN "LICENSE.txt")
install(DIRECTORY proto/ DESTINATION share/${PROJECT_NAME}/proto)

if(BUILD_TESTING)
  set(_tests test_cobs test_framing test_messages test_link)
  if(ament_cmake_FOUND)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest REQUIRED)
    foreach(t ${_tests})
      ament_add_gtest(${t} test/${t}.cpp)
      target_link_libraries(${t} scorbot_protocol)
    endforeach()
  else()
    find_package(GTest REQUIRED)
    enable_testing()
    foreach(t ${_tests})
      add_executable(${t} test/${t}.cpp)
      target_link_libraries(${t} scorbot_protocol GTest::gtest)
      add_test(NAME ${t} COMMAND ${t})
    endforeach()
  endif()
  target_link_libraries(test_link util)  # openpty
endif()

if(ament_cmake_FOUND)
  ament_export_include_directories(include include/scorbot_protocol/generated include/scorbot_protocol/nanopb)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_package()
else()
  install(EXPORT export_${PROJECT_NAME} DESTINATION lib/cmake/${PROJECT_NAME})
endif()
