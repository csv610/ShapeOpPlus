/**
 * ShapeOp Example: Fair Mesh Design
 * 
 * This example demonstrates surface smoothing by minimizing dihedral angles
 * between adjacent triangles using BendingConstraint with target bend 0.0.
 */

#include "Solver.h"
#include "Constraint.h"
#include <iostream>
#include <vector>
#include <random>

using namespace ShapeOp;

int main() {
    const int grid_size = 20;
    const int num_points = grid_size * grid_size;
    Matrix3X points(3, num_points);
    
    // 1. Initialize grid points with some "noise" or folds
    std::mt19937 gen(42);
    std::uniform_real_distribution<Scalar> dist(-0.1, 0.1);

    for (int y = 0; y < grid_size; ++y) {
        for (int x = 0; x < grid_size; ++x) {
            int id = y * grid_size + x;
            Scalar u = (Scalar)x / (grid_size - 1);
            Scalar v = (Scalar)y / (grid_size - 1);
            Scalar z = 0.0;
            
            // Add noise to internal points
            if (x > 0 && x < grid_size - 1 && y > 0 && y < grid_size - 1) {
                z = dist(gen);
            }
            
            points.col(id) = Vector3(u, v, z);
        }
    }

    Solver solver;
    solver.setPoints(points);

    // 2. Add BendingConstraint to all adjacent triangle pairs
    // We use the same triangulation as in minimal_surface.cpp
    Scalar bending_weight = 1.0;
    
    for (int y = 0; y < grid_size - 1; ++y) {
        for (int x = 0; x < grid_size - 1; ++x) {
            // Indices of the current quad
            int i00 = y * grid_size + x;
            int i10 = y * grid_size + (x + 1);
            int i01 = (y + 1) * grid_size + x;
            int i11 = (y + 1) * grid_size + (x + 1);

            // Bending across the diagonal edge (i00, i11)
            // Triangles: (i00, i10, i11) and (i00, i11, i01)
            // shared: i00, i11. opposite: i10, i01
            solver.addConstraint(std::make_shared<BendingConstraint>(std::vector<int>{i00, i11, i10, i01}, bending_weight, points, 0.0, 0.0));

            // Bending across horizontal edge (i10, i11) with quad to the right
            if (x < grid_size - 2) {
                int i21 = (y + 1) * grid_size + (x + 2);
                // Triangle 1 (right side of current quad): (i10, i20, i21) -- wait, my triangulation:
                // Quad(x+1, y): (i10, i20, i21) and (i10, i21, i11)
                // Shared edge: (i10, i11). 
                // T1 (from Quad x): (i00, i10, i11)
                // T2 (from Quad x+1): (i10, i21, i11)
                // Shared: i10, i11. opposite: i00, i21
                solver.addConstraint(std::make_shared<BendingConstraint>(std::vector<int>{i10, i11, i00, i21}, bending_weight, points, 0.0, 0.0));
            }

            // Bending across vertical edge (i01, i11) with quad above
            if (y < grid_size - 2) {
                int i12 = (y + 2) * grid_size + (x + 1);
                // Quad(x, y+1): (i01, i11, i12) and (i01, i12, i02)
                // Shared edge: (i01, i11).
                // T1 (from Quad y): (i00, i11, i01) -- wait, (i00, i11, i01)
                // T2 (from Quad y+1): (i01, i11, i12)
                // Shared: i01, i11. opposite: i00, i12
                solver.addConstraint(std::make_shared<BendingConstraint>(std::vector<int>{i01, i11, i00, i12}, bending_weight, points, 0.0, 0.0));
            }
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
    auto calculate_total_bending_energy = [&](const Matrix3X& v) {
        Scalar total = 0;
        int count = 0;
        for (int y = 1; y < grid_size - 1; ++y) {
            for (int x = 1; x < grid_size - 1; ++x) {
                int id = y * grid_size + x;
                // Measure displacement from zero plane (since our target is a flat plane)
                total += v(2, id) * v(2, id);
                count++;
            }
        }
        return total / count; // Mean Squared Error from flat
    };

    Scalar initial_energy = calculate_total_bending_energy(points);
    solver.initialize();
    std::cout << "Smoothing mesh..." << std::endl;
    solver.solve(50);

    Scalar final_energy = calculate_total_bending_energy(solver.getPoints());

    std::cout << "\n--- Optimization Metrics ---" << std::endl;
    std::cout << "Metric               | Value" << std::endl;
    std::cout << "------------------------------------------" << std::endl;
    printf("Initial MSE (Noise)  | %.6f\n", initial_energy);
    printf("Final MSE (Noise)    | %.6f\n", final_energy);
    printf("Reduction (%%)        | %.2f%%\n", (1.0 - final_energy/initial_energy)*100.0);
    std::cout << "------------------------------------------" << std::endl;

    return 0;
}
