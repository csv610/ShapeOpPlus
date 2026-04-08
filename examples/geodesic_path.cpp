/**
 * ShapeOp Example: Geodesic Path
 * 
 * This example demonstrates how to find a geodesic path on a sphere.
 * We use SphereConstraint to keep points on a sphere and 
 * EdgeStrainConstraint with target length 0.0 to pull the rope tight.
 */

#include "Solver.h"
#include "Constraint.h"
#include <iostream>
#include <vector>
#include <cmath>

using namespace ShapeOp;

int main() {
    const int num_points = 50;
    Matrix3X points(3, num_points);
    
    // 1. Initialize points on a sphere (with some noise)
    // We'll create a path from (1,0,0) to (0,1,0)
    for (int i = 0; i < num_points; ++i) {
        Scalar t = (Scalar)i / (num_points - 1);
        Scalar theta = t * M_PI / 2.0;
        
        // Initial path: a bit of a hump to not start as a geodesic
        Scalar x = cos(theta);
        Scalar y = sin(theta);
        Scalar z = 0.5 * sin(t * M_PI); // Noise/hump
        
        points.col(i) = Vector3(x, y, z);
    }

    Solver solver;
    solver.setPoints(points);

    // 2. Add SphereConstraint to keep ALL points on the same sphere
    std::vector<int> all_ids(num_points);
    for (int i = 0; i < num_points; ++i) all_ids[i] = i;
    solver.addConstraint(std::make_shared<SphereConstraint>(all_ids, 1.0, points));

    // 3. Add EdgeStrainConstraint with target length 0.0 to pull the rope
    Scalar edge_weight = 1.0;
    for (int i = 0; i < num_points - 1; ++i) {
        std::vector<int> edge_ids = {i, i + 1};
        // To force length to 0, we can use rangeMin=0, rangeMax=0
        solver.addConstraint(std::make_shared<EdgeStrainConstraint>(edge_ids, edge_weight, points, 0.0, 0.0));
    }

    // 4. Fix start and end points
    Scalar fix_weight = 100.0;
    std::vector<int> start_id = {0};
    std::vector<int> end_id = {num_points - 1};
    solver.addConstraint(std::make_shared<ClosenessConstraint>(start_id, fix_weight, points));
    solver.addConstraint(std::make_shared<ClosenessConstraint>(end_id, fix_weight, points));

    // 5. Initialize and solve
    auto calculate_path_metrics = [&](const Matrix3X& v) {
        Scalar length = 0;
        Scalar rms_surface_error = 0;
        for (int i = 0; i < num_points; ++i) {
            if (i < num_points - 1) length += (v.col(i+1) - v.col(i)).norm();
            Scalar r = v.col(i).norm();
            rms_surface_error += (r - 1.0) * (r - 1.0);
        }
        return std::make_pair(length, std::sqrt(rms_surface_error / num_points));
    };

    auto initial_metrics = calculate_path_metrics(points);
    solver.initialize();
    std::cout << "Finding geodesic path..." << std::endl;
    solver.solve(100);

    auto final_metrics = calculate_path_metrics(solver.getPoints());

    std::cout << "\n--- Optimization Metrics ---" << std::endl;
    std::cout << "Metric               | Value" << std::endl;
    std::cout << "------------------------------------------" << std::endl;
    printf("Initial Path Length  | %.6f\n", initial_metrics.first);
    printf("Final Path Length    | %.6f\n", final_metrics.first);
    printf("RMS Surface Error    | %.6f\n", final_metrics.second);
    printf("Reduction (%%)        | %.2f%%\n", (1.0 - final_metrics.first/initial_metrics.first)*100.0);
    std::cout << "------------------------------------------" << std::endl;

    return 0;
}
