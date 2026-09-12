cmake_minimum_required(VERSION 3.16)
project(scorbot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Every top-level xacro must expand to a valid URDF for every robot type.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_xacro_expands test/test_xacro_expands.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
