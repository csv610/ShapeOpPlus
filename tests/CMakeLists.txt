include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
# For Windows: do not override the global/static settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

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
