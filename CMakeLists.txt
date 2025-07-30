cmake_minimum_required(VERSION 3.5)
project(safety_module)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/safety_module_modbus_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

