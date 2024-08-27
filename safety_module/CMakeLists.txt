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
  DIRECTORY launch tables
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ARG_TESTNAME "pylint")
  set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.xunit.xml")
  set(pylint_rc_file "${CMAKE_CURRENT_SOURCE_DIR}/.pylintrc")
  set(cmd "python3" "-m" "pytest" "-p" "no:cacheprovider" "--pylint"
          "--junitxml" "${result_file}"
          "--pylint-rcfile" "${pylint_rc_file}")
  ament_add_test(
    "${ARG_TESTNAME}"
    COMMAND ${cmd}
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_pep257/${ARG_TESTNAME}.txt"
    RESULT_FILE "${result_file}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  set_tests_properties(
    "${ARG_TESTNAME}"
    PROPERTIES
    LABELS "pylint;linter"
  )
endif()

ament_package()
