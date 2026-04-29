find_package(GTest 1.17 REQUIRED)
enable_testing()

add_executable(unit_tests
  test_solver.cpp
  test_constraints.cpp
)

target_include_directories(unit_tests PRIVATE
  ${CMAKE_SOURCE_DIR}/libShapeOp/src
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(unit_tests
  GTest::gtest_main
  ShapeOp
)

include(GoogleTest)
gtest_discover_tests(unit_tests)
