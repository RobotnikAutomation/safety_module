cmake_minimum_required(VERSION 3.16)
project(safety_module)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
#find_package(generate_parameter_library REQUIRED)

#generate_parameter_module(safety_module_parameters
#  safety_module/parameters.yaml
#)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/safety_module_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config tables
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
