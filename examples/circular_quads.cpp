/**
 * ShapeOp Example: Cyclic Quadrilateral Mesh Regularization
 * 
 * This example demonstrates how to force every quadrilateral element in a 
 * 2D mesh to be "cyclic" (meaning all 4 vertices lie on a common circle).
 * 
 * Concepts covered:
 * 1. Defining a quadrilateral mesh.
 * 2. Applying CircleConstraint to sets of 4 vertices.
 * 3. Fixing boundaries to preserve the global mesh layout.
 */

#include "Solver.h"
#include "Constraint.h"
#include <iostream>
#include <vector>
#include <iomanip>

using namespace ShapeOp;

int main() {
    // 1. Create a 3x3 grid of points (forming 4 quads)
    const int grid_res = 3;
    const int num_points = grid_res * grid_res;
    Matrix3X points(3, num_points);
    
    for (int y = 0; y < grid_res; ++y) {
        for (int x = 0; x < grid_res; ++x) {
            int id = y * grid_res + x;
            // Add noise to initial positions to ensure they are not cyclic
            Scalar noise_x = (id == 4) ? 0.2 : 0.0; // Offset the center point
            Scalar noise_y = (id == 4) ? -0.1 : 0.0;
            points.col(id) = Vector3(x + noise_x, y + noise_y, 0.0);
        }
    }

    // Define Quad indices (counter-clockwise)
    struct Quad { int v0, v1, v2, v3; };
    std::vector<Quad> quads;
    for (int y = 0; y < grid_res - 1; ++y) {
        for (int x = 0; x < grid_res - 1; ++x) {
            int p0 = y * grid_res + x;
            int p1 = p0 + 1;
            int p2 = (y + 1) * grid_res + x + 1;
            int p3 = (y + 1) * grid_res + x;
            quads.push_back({p0, p1, p2, p3});
        }
    }

    // 2. Setup Solver and Constraints
    Solver solver;
    solver.setPoints(points);

    // Add CircleConstraint to each quad
    Scalar weight = 1.0;
    for (const auto& q : quads) {
        std::vector<int> ids = {q.v0, q.v1, q.v2, q.v3};
        solver.addConstraint(std::make_shared<CircleConstraint>(ids, weight, points));
    }

    // 3. Anchor the boundary points strictly (weight 1000.0)
    for (int i = 0; i < num_points; ++i) {
        int x = i % grid_res;
        int y = i / grid_res;
        if (x == 0 || x == grid_res - 1 || y == 0 || y == grid_res - 1) {
            solver.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{i}, 1000.0, points));
        }
    }

    // 4. Solve
    std::cout << "Regularizing quads to be cyclic (all 4 vertices on a circle)..." << std::endl;
    solver.initialize();
    solver.solve(100);

    const Matrix3X& final_points = solver.getPoints();
    
    // 5. Verification
    // For each quad, the distance from each vertex to the quad's centroid 
    // should be roughly equal (radius).
    std::cout << std::fixed << std::setprecision(4);
    for (size_t i = 0; i < quads.size(); ++i) {
        const auto& q = quads[i];
        Vector3 center = (final_points.col(q.v0) + final_points.col(q.v1) + 
                          final_points.col(q.v2) + final_points.col(q.v3)) / 4.0;
        
        std::cout << "\nQuad " << i << " radial distances from centroid:" << std::endl;
        std::cout << "  P" << q.v0 << ": " << (final_points.col(q.v0) - center).norm() << std::endl;
        std::cout << "  P" << q.v1 << ": " << (final_points.col(q.v1) - center).norm() << std::endl;
        std::cout << "  P" << q.v2 << ": " << (final_points.col(q.v2) - center).norm() << std::endl;
        std::cout << "  P" << q.v3 << ": " << (final_points.col(q.v3) - center).norm() << std::endl;
    }

    // Check boundary displacement
    Scalar boundary_displacement = (final_points.col(0) - points.col(0)).norm();
    std::cout << "\nBoundary Point P0 displacement: " << boundary_displacement << " (should be ~0)" << std::endl;

    return 0;
}
