add_executable(cloth_simulation cloth_simulation.cpp)
target_include_directories(cloth_simulation PRIVATE
  ${CMAKE_SOURCE_DIR}/libShapeOp/src
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(cloth_simulation ShapeOp)

add_executable(capabilities_showcase capabilities_showcase.cpp)
target_include_directories(capabilities_showcase PRIVATE
  ${CMAKE_SOURCE_DIR}/libShapeOp/src
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(capabilities_showcase ShapeOp)
