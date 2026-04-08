/**
 * ShapeOp Example: Quad Planarization
 * 
 * This example demonstrates how to force all faces of a quadrilateral mesh
 * to be planar using PlaneConstraint.
 */

#include "Solver.h"
#include "Constraint.h"
#include <iostream>
#include <vector>

using namespace ShapeOp;

int main() {
    const int grid_size = 10;
    const int num_points = grid_size * grid_size;
    Matrix3X points(3, num_points);
    
    // 1. Initialize a curved quad mesh (hyperbolic paraboloid)
    for (int y = 0; y < grid_size; ++y) {
        for (int x = 0; x < grid_size; ++x) {
            int id = y * grid_size + x;
            Scalar u = (Scalar)x / (grid_size - 1);
            Scalar v = (Scalar)y / (grid_size - 1);
            // z = u * v is a doubly-ruled surface but its quads are non-planar
            Scalar z = 2.0 * u * v; 
            points.col(id) = Vector3(u, v, z);
        }
    }

    Solver solver;
    solver.setPoints(points);

    // 2. Add PlaneConstraint to every quad
    Scalar plane_weight = 10.0;
    for (int y = 0; y < grid_size - 1; ++y) {
        for (int x = 0; x < grid_size - 1; ++x) {
            int i00 = y * grid_size + x;
            int i10 = y * grid_size + (x + 1);
            int i01 = (y + 1) * grid_size + x;
            int i11 = (y + 1) * grid_size + (x + 1);

            // Quad vertices: i00, i10, i11, i01
            std::vector<int> quad_ids = {i00, i10, i11, i01};
            solver.addConstraint(std::make_shared<PlaneConstraint>(quad_ids, plane_weight, points));
        }
    }

    // 3. Fix boundary nodes
    Scalar fix_weight = 100.0;
    for (int y = 0; y < grid_size; ++y) {
        for (int x = 0; x < grid_size; ++x) {
            if (x == 0 || x == grid_size - 1 || y == 0 || y == grid_size - 1) {
                int id = y * grid_size + x;
                std::vector<int> ids = {id};
                solver.addConstraint(std::make_shared<ClosenessConstraint>(ids, fix_weight, points));
            }
        }
    }

    // 4. Initialize and solve
    auto calculate_planarity_error = [&](const Matrix3X& v) {
        Scalar total_error = 0;
        int count = 0;
        for (int y = 0; y < grid_size - 1; ++y) {
            for (int x = 0; x < grid_size - 1; ++x) {
                int i00 = y * grid_size + x;
                int i10 = y * grid_size + (x + 1);
                int i01 = (y + 1) * grid_size + x;
                int i11 = (y + 1) * grid_size + (x + 1);
                
                // Plane defined by i00, i10, i01
                Vector3 p0 = v.col(i00);
                Vector3 p1 = v.col(i10);
                Vector3 p2 = v.col(i01);
                Vector3 p3 = v.col(i11);
                Vector3 normal = (p1 - p0).cross(p2 - p0).normalized();
                
                // Distance of p3 to this plane
                Scalar dist = std::abs(normal.dot(p3 - p0));
                total_error += dist;
                count++;
            }
        }
        return total_error / count; // Mean absolute error from planarity
    };

    Scalar initial_error = calculate_planarity_error(points);
    solver.initialize();
    std::cout << "Planarizing quad mesh..." << std::endl;
    solver.solve(500);

    Scalar final_error = calculate_planarity_error(solver.getPoints());

    std::cout << "\n--- Optimization Metrics ---" << std::endl;
    std::cout << "Metric               | Value" << std::endl;
    std::cout << "------------------------------------------" << std::endl;
    printf("Initial Planar Error | %.6f\n", initial_error);
    printf("Final Planar Error   | %.6f\n", final_error);
    printf("Reduction (%%)        | %.2f%%\n", (1.0 - final_error/initial_error)*100.0);
    std::cout << "------------------------------------------" << std::endl;

    return 0;
}
