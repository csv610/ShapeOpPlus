/**
 * ShapeOp Example: Equilateral Triangle Regularization
 * 
 * This example demonstrates how to:
 * 1. Define a 2D triangle mesh.
 * 2. Create a "target" equilateral triangle shape.
 * 3. Use SimilarityConstraint to force every triangle in the mesh to 
 *    match the equilateral target (allowing for rotation, translation, and scaling).
 * 4. Fix boundary points to maintain the overall domain.
 */

#include "Solver.h"
#include "Constraint.h"
#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath>

using namespace ShapeOp;

// Helper to calculate the quality of a triangle (how close to equilateral it is)
// We use the ratio of area to squared edge length sum as a simple metric.
Scalar calculate_equilateral_quality(const Vector3& p0, const Vector3& p1, const Vector3& p2) {
    Scalar a = (p1 - p0).norm();
    Scalar b = (p2 - p1).norm();
    Scalar c = (p0 - p2).norm();
    Vector3 v1 = p1 - p0;
    Vector3 v2 = p2 - p0;
    Scalar area = 0.5 * v1.cross(v2).norm();
    // For equilateral, area / (a^2+b^2+c^2) is approx 0.0481 (sqrt(3)/36)
    // We'll just return the area for debugging.
    return area;
}

int main() {
    // 1. Create a 2D mesh: a 3x3 grid
    const int grid_res = 3;
    const int num_points = grid_res * grid_res;
    Matrix3X points(3, num_points);
    
    for (int y = 0; y < grid_res; ++y) {
        for (int x = 0; x < grid_res; ++x) {
            int id = y * grid_res + x;
            // Add significant noise to make triangles non-equilateral
            Scalar noise_x = (x == 1 && y == 1) ? 0.4 : 0.0;
            Scalar noise_y = (x == 1 && y == 1) ? -0.3 : 0.0;
            points.col(id) = Vector3(x + noise_x, y + noise_y, 0.0);
        }
    }

    // Define triangle indices
    struct Triangle { int v0, v1, v2; };
    std::vector<Triangle> triangles;
    for (int y = 0; y < grid_res - 1; ++y) {
        for (int x = 0; x < grid_res - 1; ++x) {
            int p0 = y * grid_res + x;
            int p1 = p0 + 1;
            int p2 = (y + 1) * grid_res + x;
            int p3 = p2 + 1;
            triangles.push_back({p0, p1, p3});
            triangles.push_back({p0, p3, p2});
        }
    }

    // 2. Define the Target Shape: A perfect equilateral triangle
    // Sides of length 1.0
    Matrix3X target_equilateral(3, 3);
    target_equilateral << 0.0, 1.0, 0.5,
                          0.0, 0.0, std::sqrt(3.0) / 2.0,
                          0.0, 0.0, 0.0;
    std::vector<Matrix3X> target_shapes = {target_equilateral};

    // 3. Setup Solver and Constraints
    Solver solver;
    solver.setPoints(points);

    // Add SimilarityConstraint to each triangle
    for (const auto& t : triangles) {
        std::vector<int> ids = {t.v0, t.v1, t.v2};
        // allowScaling = true (Similarity), weight = 1.0
        auto sim_cons = std::make_shared<SimilarityConstraint>(ids, 1.0, points, true);
        sim_cons->setShapes(target_shapes);
        solver.addConstraint(sim_cons);
    }

    // 4. Anchor the boundary points
    for (int i = 0; i < num_points; ++i) {
        int x = i % grid_res;
        int y = i / grid_res;
        if (x == 0 || x == grid_res - 1 || y == 0 || y == grid_res - 1) {
            solver.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{i}, 10.0, points));
        }
    }

    // 5. Solve
    std::cout << "Regularizing mesh to equilateral triangles..." << std::endl;
    solver.initialize();
    solver.solve(50);

    const Matrix3X& final_points = solver.getPoints();
    
    std::cout << "Final Positions of internal point (P4):" << std::endl;
    std::cout << final_points.col(4).transpose() << std::endl;
    
    std::cout << "\nEdge lengths of the first triangle (should be nearly equal):" << std::endl;
    const auto& t = triangles[0];
    Vector3 p0 = final_points.col(t.v0);
    Vector3 p1 = final_points.col(t.v1);
    Vector3 p2 = final_points.col(t.v2);
    std::cout << "L1: " << (p1-p0).norm() << std::endl;
    std::cout << "L2: " << (p2-p1).norm() << std::endl;
    std::cout << "L3: " << (p0-p2).norm() << std::endl;

    return 0;
}
