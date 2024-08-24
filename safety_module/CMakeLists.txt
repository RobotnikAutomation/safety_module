cmake_minimum_required(VERSION 3.16)
project(safety_module)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/safety_module_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config tables
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_test(
    NAME pylint
    COMMAND /usr/bin/python3 -m pytest --pylint --pylint-rcfile=/home/robot/robot_ws/src/safety_module/.pylintrc --junitxml=/home/robot/robot_ws/reports/xunit-result.xml /home/robot/robot_ws/src/safety_module
  )
  # find_package(ament_cmake_pytest REQUIRED)
endif()

ament_package()
