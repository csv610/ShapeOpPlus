# ShapeOp Examples

This directory contains examples of how to use the ShapeOp library in both C++ and Python.

## 1. Cloth Simulation (C++)
A 3x3 grid cloth simulation that demonstrates:
- Grid initialization.
- Adding `EdgeStrain` constraints (structural springs).
- Fixing points in space using `ClosenessConstraint`.
- Applying `GravityForce`.
- Running a dynamic simulation with time integration.

**To run:**
From the project root (after building):
```bash
./examples/cloth_simulation
```

## 2. Catenary Curve (Python)
A 10-point rope "hanging" between two fixed points, demonstrating:
- Use of the Python bindings.
- Static-to-Dynamic initialization.
- Editing constraint parameters at runtime.

**To run:**
From the project root:
```bash
PYTHONPATH=libShapeOp/bindings/python python3 examples/catenary_curve.py
```

## 3. Capabilities Showcase (C++)
A comprehensive C++ executable that demonstrates several advanced features of the ShapeOp library in isolated scenarios:
- **Laplacian Smoothing:** Using `UniformLaplacianConstraint` to flatten a bump in a line of points.
- **Volume Preservation:** Using `VolumeConstraint` to maintain the volume of a tetrahedron while pulling its apex upwards.
- **Rigid/Similarity Matching:** Using `SimilarityConstraint` to force points to match a target geometric shape.
- **Geometric Primitive Fitting:** Using `CircleConstraint` to fit points to a circle.
- **Custom Solvers & Forces:** Swapping the linear solver to `CGSolver` and applying a `VertexForce` to pull a specific point.

**To run:**
From the project root (after building):
```bash
./examples/capabilities_showcase
```

## 4. Triangle Area Equalization (C++)
Demonstrates regularizing a 2D mesh so that all triangles have equal area:
- Calculates the initial average area of the mesh.
- Uses `AreaConstraint` with dynamic target ratios to force each triangle to the average.
- Anchors the boundary points to maintain the overall geometry.

**To run:**
From the project root (after building):
```bash
./examples/area_equalization
```

## 5. Equilateral Triangle Regularization (C++)
Demonstrates how to force triangles in a 2D mesh to become as equilateral as possible:
- Defines a perfect equilateral triangle as a target shape.
- Uses `SimilarityConstraint` on every triangle to match the target shape (allowing for rotation and uniform scaling).
- Shows how the solver balances local shape goals with global boundary constraints.

**To run:**
From the project root (after building):
```bash
./examples/equilateral_regularization
```

## 6. Cyclic Quadrilateral Regularization (C++)
Demonstrates forcing every quadrilateral in a mesh to be "cyclic" (all 4 vertices on a common circle):
- Applies `CircleConstraint` to the 4 vertices of each quad element.
- Anchors boundary nodes to preserve the global structure.
- Verifies the results by checking the radial distance from each vertex to its quad's centroid.

**To run:**
From the project root (after building):
```bash
./examples/circular_quads
```
1. **Include Headers:** Use `Solver.h`, `Constraint.h`, and `Force.h`.
2. **Define Points:** Provide point coordinates as a `Matrix3X`.
3. **Add Constraints:** Use `solver.addConstraint()` with various constraint types.
4. **Initialize:** Call `solver.initialize()` for static problems or `solver.initialize(true, ...)` for dynamics.
5. **Solve:** Call `solver.solve(iterations)` in your main loop.
