/**
 * ShapeOp Example: Triangle Area Equalization
 * 
 * This example demonstrates how to:
 * 1. Read a triangle mesh from an OFF file.
 * 2. Calculate the average area of all triangles.
 * 3. Use AreaConstraint to force all triangles towards that average area.
 * 4. Fix boundary points to maintain the overall shape.
 */

#include "common.h"
#include "Constraint.h"
#include <iostream>
#include <vector>
#include <iomanip>
#include <numeric>

using namespace ShapeOp;

Scalar calculate_triangle_area(const Vector3& p0, const Vector3& p1, const Vector3& p2) {
    Vector3 v1 = p1 - p0;
    Vector3 v2 = p2 - p0;
    return 0.5 * v1.cross(v2).norm();
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " input.off [output.off]" << std::endl;
        return 1;
    }

    std::string input_file = argv[1];
    std::string output_file = (argc > 2) ? argv[2] : "out.off";

    Mesh mesh;
    if (!readOFF(input_file, mesh)) {
        std::cerr << "Error: Could not read " << input_file << std::endl;
        return 1;
    }

    // Check if all faces are triangles
    for (const auto& face : mesh.faces) {
        if (face.size() != 3) {
            std::cerr << "Error: Area equalization example requires a triangle mesh." << std::endl;
            return 1;
        }
    }

    std::vector<Scalar> initial_areas;
    Scalar total_area = 0;
    for (const auto& t : mesh.faces) {
        Scalar a = calculate_triangle_area(mesh.vertices.col(t[0]), mesh.vertices.col(t[1]), mesh.vertices.col(t[2]));
        initial_areas.push_back(a);
        total_area += a;
    }
    Scalar avg_area = total_area / mesh.faces.size();

    Solver solver;
    solver.setPoints(mesh.vertices);

    for (size_t i = 0; i < mesh.faces.size(); ++i) {
        const auto& t = mesh.faces[i];
        std::vector<int> ids = {t[0], t[1], t[2]};
        Scalar target_ratio = avg_area / initial_areas[i];
        auto area_cons = std::make_shared<AreaConstraint>(ids, 1.0, mesh.vertices, target_ratio, target_ratio);
        solver.addConstraint(area_cons);
    }

    std::set<int> boundary = findBoundaryVertices(mesh);
    for (int idx : boundary) {
        solver.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{idx}, 1000.0, mesh.vertices));
    }

    solver.initialize();
    solver.solve(100); 

    mesh.vertices = solver.getPoints();
    writeOFF(output_file, mesh);
    std::cout << "Saved results to " << output_file << std::endl;

    return 0;
}
