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
    
    // 1. Initialize a curved quad mesh (e.g., on a paraboloid)
    for (int y = 0; y < grid_size; ++y) {
        for (int x = 0; x < grid_size; ++x) {
            int id = y * grid_size + x;
            Scalar u = (Scalar)x / (grid_size - 1);
            Scalar v = (Scalar)y / (grid_size - 1);
            Scalar z = 0.5 * (u * u + v * v);
            points.col(id) = Vector3(u, v, z);
        }
    }

    Solver solver;
    solver.setPoints(points);

    // 2. Add PlaneConstraint to every quad
    Scalar plane_weight = 1.0;
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
    solver.initialize();
    std::cout << "Planarizing quad mesh..." << std::endl;
    solver.solve(100);

    std::cout << "Quads planarized." << std::endl;

    return 0;
}
