cmake_minimum_required(VERSION 2.8.3)
project(safety_module)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  robotnik_msgs
  sick_safetyscanners
)


catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS src/safety_module/__init__.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS src/safety_module/safety_module_modbus_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS src/safety_module/safety_module_io_nanoscan_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
