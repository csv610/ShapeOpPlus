/**
 * ShapeOp Example: Minimal Surface
 * 
 * This example demonstrates how to find a minimal surface (soap film)
 * given a fixed boundary. We use AreaConstraint with target area 0.0
 * on all triangles.
 */

#include "Solver.h"
#include "Constraint.h"
#include <iostream>
#include <vector>

using namespace ShapeOp;

int main() {
    const int grid_size = 20;
    const int num_points = grid_size * grid_size;
    Matrix3X points(3, num_points);
    
    // 1. Initialize grid points with a saddle-shaped boundary
    for (int y = 0; y < grid_size; ++y) {
        for (int x = 0; x < grid_size; ++x) {
            int id = y * grid_size + x;
            Scalar u = (Scalar)x / (grid_size - 1);
            Scalar v = (Scalar)y / (grid_size - 1);
            
            // Saddle: z = (x-0.5)^2 - (y-0.5)^2
            Scalar z = (u - 0.5) * (u - 0.5) - (v - 0.5) * (v - 0.5);
            
            // For internal points, start at z=0 (or anywhere)
            if (x > 0 && x < grid_size - 1 && y > 0 && y < grid_size - 1) {
                z = 0.0;
            }
            
            points.col(id) = Vector3(u, v, z);
        }
    }

    Solver solver;
    solver.setPoints(points);

    // 2. Add AreaConstraint to all triangles with target area 0.0
    // We set rangeMin and rangeMax to 0.0 to force the area to zero.
    Scalar area_weight = 1.0;
    for (int y = 0; y < grid_size - 1; ++y) {
        for (int x = 0; x < grid_size - 1; ++x) {
            int i00 = y * grid_size + x;
            int i10 = y * grid_size + (x + 1);
            int i01 = (y + 1) * grid_size + x;
            int i11 = (y + 1) * grid_size + (x + 1);

            // Triangle 1
            std::vector<int> tri1 = {i00, i10, i11};
            solver.addConstraint(std::make_shared<AreaConstraint>(tri1, area_weight, points, 0.0, 0.0));

            // Triangle 2
            std::vector<int> tri2 = {i00, i11, i01};
            solver.addConstraint(std::make_shared<AreaConstraint>(tri2, area_weight, points, 0.0, 0.0));
        }
    }

    // 3. Fix boundary nodes using ClosenessConstraint
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
    std::cout << "Solving for minimal surface..." << std::endl;
    solver.solve(100); // 100 iterations for convergence

    const Matrix3X& result = solver.getPoints();
    std::cout << "Convergence reached." << std::endl;

    // Output some results
    std::cout << "Center point (z): " << result(2, (grid_size/2) * grid_size + (grid_size/2)) << std::endl;

    return 0;
}
