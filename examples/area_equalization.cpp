/**
 * ShapeOp Example: 2D Triangle Area Equalization
 * 
 * This example demonstrates how to:
 * 1. Define a 2D triangle mesh.
 * 2. Calculate the average area of all triangles.
 * 3. Use AreaConstraint with custom range scaling to force all triangles
 *    towards that average area.
 * 4. Fix boundary points to maintain the overall shape.
 */

#include "Solver.h"
#include "Constraint.h"
#include <iostream>
#include <vector>
#include <iomanip>
#include <numeric>

using namespace ShapeOp;

// Helper to calculate triangle area in 3D (assuming they lie in XY plane or similar)
Scalar calculate_triangle_area(const Vector3& p0, const Vector3& p1, const Vector3& p2) {
    Vector3 v1 = p1 - p0;
    Vector3 v2 = p2 - p0;
    return 0.5 * v1.cross(v2).norm();
}

int main() {
    // 1. Create a 2D mesh: a 3x3 grid of points forming 8 triangles
    // Initially, let's make it irregular to show the equalization
    const int grid_res = 3;
    const int num_points = grid_res * grid_res;
    Matrix3X points(3, num_points);
    
    for (int y = 0; y < grid_res; ++y) {
        for (int x = 0; x < grid_res; ++x) {
            int id = y * grid_res + x;
            // Add some "noise" to the initial positions to make areas unequal
            Scalar noise_x = (x == 1 && y == 1) ? 0.3 : 0.0;
            Scalar noise_y = (x == 1 && y == 1) ? -0.2 : 0.0;
            points.col(id) = Vector3(x + noise_x, y + noise_y, 0.0);
        }
    }

    // Define triangle indices (2 triangles per square cell)
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

    // 2. Calculate initial areas and find the average
    std::vector<Scalar> initial_areas;
    Scalar total_area = 0;
    for (const auto& t : triangles) {
        Scalar a = calculate_triangle_area(points.col(t.v0), points.col(t.v1), points.col(t.v2));
        initial_areas.push_back(a);
        total_area += a;
    }
    Scalar avg_area = total_area / triangles.size();

    std::cout << "Initial average area: " << avg_area << std::endl;
    std::cout << "Initial individual areas: ";
    for (Scalar a : initial_areas) std::cout << std::fixed << std::setprecision(3) << a << " ";
    std::cout << "\n" << std::endl;

    // 3. Setup Solver and Constraints
    Solver solver;
    solver.setPoints(points);

    // Add AreaConstraint to each triangle
    // AreaConstraint in ShapeOp uses rangeMin/rangeMax as factors for det(F).
    // det(F) = CurrentArea / InitialArea.
    // To reach TargetArea, we want det(F) = TargetArea / InitialArea.
    for (size_t i = 0; i < triangles.size(); ++i) {
        const auto& t = triangles[i];
        std::vector<int> ids = {t.v0, t.v1, t.v2};
        Scalar target_ratio = avg_area / initial_areas[i];
        
        // Setting rangeMin and rangeMax to the same value forces the area to exactly that ratio
        auto area_cons = std::make_shared<AreaConstraint>(ids, 1.0, points, target_ratio, target_ratio);
        solver.addConstraint(area_cons);
    }

    // 4. Anchor the boundary points to prevent the whole mesh from collapsing/drifting
    for (int i = 0; i < num_points; ++i) {
        int x = i % grid_res;
        int y = i / grid_res;
        if (x == 0 || x == grid_res - 1 || y == 0 || y == grid_res - 1) {
            solver.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{i}, 10.0, points));
        }
    }

    // 5. Solve
    solver.initialize();
    solver.solve(100); // 100 iterations for high precision

    const Matrix3X& final_points = solver.getPoints();
    
    // Calculate final areas
    std::cout << "Final individual areas (should be close to " << avg_area << "):" << std::endl;
    for (const auto& t : triangles) {
        Scalar a = calculate_triangle_area(final_points.col(t.v0), final_points.col(t.v1), final_points.col(t.v2));
        std::cout << std::fixed << std::setprecision(3) << a << " ";
    }
    std::cout << std::endl;

    return 0;
}
