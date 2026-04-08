/**
 * ShapeOp Example: 3x3 Cloth Grid Simulation
 * 
 * This example demonstrates how to:
 * 1. Initialize a grid of points.
 * 2. Add structural "Edge" constraints (springs).
 * 3. Add "Closeness" constraints to fix points in space.
 * 4. Apply external forces (Gravity).
 * 5. Run a dynamic (time-integrated) simulation.
 */

#include "Solver.h"
#include "Constraint.h"
#include "Force.h"
#include <iostream>
#include <iomanip>

using namespace ShapeOp;

int main() {
    const int grid_size = 3;
    const int num_points = grid_size * grid_size;
    Matrix3X points(3, num_points);
    
    // 1. Create a 3x3 horizontal grid in the XY plane
    for (int y = 0; y < grid_size; ++y) {
        for (int x = 0; x < grid_size; ++x) {
            int id = y * grid_size + x;
            points.col(id) = Vector3(x, y, 0.0);
        }
    }

    Solver solver;
    solver.setPoints(points);

    // 2. Add structural constraints (Edges)
    Scalar weight = 1.0;
    for (int y = 0; y < grid_size; ++y) {
        for (int x = 0; x < grid_size; ++x) {
            int id = y * grid_size + x;
            // Horizontal edge
            if (x + 1 < grid_size) {
                std::vector<int> ids = {id, id + 1};
                solver.addConstraint(std::make_shared<EdgeStrainConstraint>(ids, weight, points));
            }
            // Vertical edge
            if (y + 1 < grid_size) {
                std::vector<int> ids = {id, id + grid_size};
                solver.addConstraint(std::make_shared<EdgeStrainConstraint>(ids, weight, points));
            }
        }
    }

    // 3. Fix the top two corners (y=2, x=0 and x=2)
    std::vector<int> top_left = { (grid_size-1) * grid_size + 0 };
    std::vector<int> top_right = { (grid_size-1) * grid_size + (grid_size-1) };
    
    solver.addConstraint(std::make_shared<ClosenessConstraint>(top_left, 10.0, points));
    solver.addConstraint(std::make_shared<ClosenessConstraint>(top_right, 10.0, points));

    // 4. Add Gravity (pointing in -Z direction)
    // Note: ShapeOp defines forces as accelerations.
    solver.addForces(std::make_shared<GravityForce>(Vector3(0.0, 0.0, -9.81)));

    // 5. Initialize the dynamic solver
    // Parameters: dynamic=true, masses=1.0, damping=0.95, timestep=0.1
    solver.initialize(true, 1.0, 0.95, 0.1);

    std::cout << "Starting Simulation (10 steps)..." << std::endl;
    std::cout << std::fixed << std::setprecision(3);

    for (int i = 0; i < 10; ++i) {
        solver.solve(10); // Run 10 internal optimization iterations per timestep
        const Matrix3X& p = solver.getPoints();
        
        // Output the Z-coordinate of the center point (1,1)
        std::cout << "Step " << i << " | Center Z: " << p(2, 4) << std::endl;
    }

    std::cout << "\nFinal Grid Positions (Z coordinates):" << std::endl;
    const Matrix3X& final_p = solver.getPoints();
    for (int y = grid_size - 1; y >= 0; --y) {
        for (int x = 0; x < grid_size; ++x) {
            std::cout << std::setw(8) << final_p(2, y * grid_size + x) << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
