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

add_executable(area_equalization area_equalization.cpp)
target_include_directories(area_equalization PRIVATE
  ${CMAKE_SOURCE_DIR}/libShapeOp/src
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(area_equalization ShapeOp)

add_executable(equilateral_regularization equilateral_regularization.cpp)
target_include_directories(equilateral_regularization PRIVATE
  ${CMAKE_SOURCE_DIR}/libShapeOp/src
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(equilateral_regularization ShapeOp)

add_executable(circular_quads circular_quads.cpp)
target_include_directories(circular_quads PRIVATE
  ${CMAKE_SOURCE_DIR}/libShapeOp/src
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(circular_quads ShapeOp)

add_executable(as_similar_as_possible_mapping as_similar_as_possible_mapping.cpp)
target_include_directories(as_similar_as_possible_mapping PRIVATE
  ${CMAKE_SOURCE_DIR}/libShapeOp/src
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(as_similar_as_possible_mapping ShapeOp)
