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

add_executable(as_rigid_as_possible_mapping as_rigid_as_possible_mapping.cpp)
target_include_directories(as_rigid_as_possible_mapping PRIVATE
  ${CMAKE_SOURCE_DIR}/libShapeOp/src
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(as_rigid_as_possible_mapping ShapeOp)

add_executable(as_conformal_as_possible_mapping as_conformal_as_possible_mapping.cpp)
target_include_directories(as_conformal_as_possible_mapping PRIVATE
  ${CMAKE_SOURCE_DIR}/libShapeOp/src
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(as_conformal_as_possible_mapping ShapeOp)

add_executable(as_area_preserving_as_possible_mapping as_area_preserving_as_possible_mapping.cpp)
target_include_directories(as_area_preserving_as_possible_mapping PRIVATE
  ${CMAKE_SOURCE_DIR}/libShapeOp/src
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(as_area_preserving_as_possible_mapping ShapeOp)

add_executable(as_smooth_as_possible_mapping as_smooth_as_possible_mapping.cpp)
target_include_directories(as_smooth_as_possible_mapping PRIVATE
  ${CMAKE_SOURCE_DIR}/libShapeOp/src
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(as_smooth_as_possible_mapping ShapeOp)

add_executable(minimal_surface minimal_surface.cpp)
target_include_directories(minimal_surface PRIVATE
  ${CMAKE_SOURCE_DIR}/libShapeOp/src
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(minimal_surface ShapeOp)

add_executable(fair_mesh_design fair_mesh_design.cpp)
target_include_directories(fair_mesh_design PRIVATE
  ${CMAKE_SOURCE_DIR}/libShapeOp/src
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(fair_mesh_design ShapeOp)

add_executable(quad_planarization quad_planarization.cpp)
target_include_directories(quad_planarization PRIVATE
  ${CMAKE_SOURCE_DIR}/libShapeOp/src
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(quad_planarization ShapeOp)

add_executable(geodesic_path geodesic_path.cpp)
target_include_directories(geodesic_path PRIVATE
  ${CMAKE_SOURCE_DIR}/libShapeOp/src
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(geodesic_path ShapeOp)
