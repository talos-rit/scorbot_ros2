cmake_minimum_required(VERSION 3.16)
project(scorbot_bringup)

find_package(ament_cmake REQUIRED)
find_package(amend_cmake_python REQUIRED)

# Python helpers used by the launch files
amend_python_install_package(${PROJECT_NAME})

install(
    DIRECTORY config launch rviz
    DESTINATION share/${PROJECT_NAME}
)

install(
    PROGRAMS scripts/demo_motion.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME demo_motion
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_controllers_config test/test_controllers_config.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
endif()

ament_package()